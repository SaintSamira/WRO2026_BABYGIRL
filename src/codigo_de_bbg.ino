#include <Servo.h>        // Library to control the steering servo
//#include <SoftwareSerial.h>
#include <Wire.h>         // I2C communication (distance sensors + Nicla camera)
#include <VL53L0X.h>      // Library for the VL53L0X time-of-flight distance sensors

// ---------------------------------------------------------------------------
//  Vision data packet sent by the Nicla Vision camera
// ---------------------------------------------------------------------------
// #pragma pack(1) removes padding between fields so the struct in memory has
// exactly the same byte layout as the packet the camera sends (18 bytes).
#pragma pack(1)
struct VisionPacket {
    uint8_t  status_flags;          // Bit flags: which objects were detected (see FLAG_* below)
    uint16_t red_cx;                // Horizontal center (x pixel) of the red pillar in the image
    uint16_t red_distance_mm;       // Estimated distance to the red pillar in millimeters
    uint16_t green_cx;              // Horizontal center (x pixel) of the green pillar
    uint16_t green_distance_mm;     // Estimated distance to the green pillar in millimeters
    uint16_t parking_cx;            // Horizontal center (x pixel) of the magenta parking slot
    uint16_t parking_distance_mm;   // Estimated distance to the parking slot (may be NOT_FOUND_SENTINEL)
    uint16_t yellow_closest_mm;     // Distance to the closest yellow crosswalk stripe
    uint16_t yellow_farthest_mm;    // Distance to the farthest yellow crosswalk stripe
    uint8_t  checksum;              // XOR of all previous bytes, used to verify the packet is not corrupted
};
#pragma pack()   // Restore the compiler's normal struct packing

// Bit masks used to read status_flags (each bit = one detected object)
#define FLAG_RED     0x01   // Bit 0: red pillar detected
#define FLAG_GREEN   0x02   // Bit 1: green pillar detected
#define FLAG_PARKING 0x04   // Bit 2: magenta parking slot detected
#define FLAG_YELLOW  0x08   // Bit 3: yellow crosswalk stripes detected
#define NOT_FOUND_SENTINEL 0xFFFF   // Special value meaning "distance not available"

#define NICLA_I2C_ADDR 8   // I2C address of the Nicla Vision camera (the old 0x12 value is no longer used)
#define PACKET_SIZE sizeof(VisionPacket)  // Size of the vision packet in bytes (must equal 18)

VisionPacket packet;   // Global copy of the vision packet (note: non_free_round() uses its own local copy)

// ---------------------------------------------------------------------------
//  Steering servo and sensor index constants
// ---------------------------------------------------------------------------
Servo myservo;         // Steering servo object
const int l = 0;       // Index of the LEFT distance sensor
const int f = 1;       // Index of the FRONT distance sensor
const int r = 2;       // Index of the RIGHT distance sensor
double Fdist = 300, Ldist = 300, Rdist = 300; // Last readings (mm) of front/left/right sensors; 300 = placeholder start values

// Servo angle references (degrees). Values were calibrated on the real robot.
const int CENTER_ANGLE  = 83;   // Wheels straight (previously 91)
const int MIN_ANGLE     = 48;   // Maximum steering to one side (about -35 from center)
const int MAX_ANGLE     = 118;  // Maximum steering to the other side (about +35 from center)

// ---------------------------------------------------------------------------
//  VL53L0X distance sensors
// ---------------------------------------------------------------------------
VL53L0X sensors[3];                                  // Array of the 3 distance sensors (left, front, right)
const int xshutPins[]     = {7, 6, 5};               // XSHUT (reset/enable) pin of each sensor: left, front, right
const uint8_t addresses[] = {0x30, 0x31, 0x32};      // Unique I2C address assigned to each sensor at startup
                                                     // (all VL53L0X start with the same address, so each one must be changed;
                                                     //  these must not conflict with other I2C devices such as the Nicla)

// ---------------------------------------------------------------------------
//  Start button and status LED
// ---------------------------------------------------------------------------
const int keyPin = A3;   // Pin where the start button is connected
const int keyLed = 13;   // LED that turns on to confirm the button was pressed
int buttonState = 0;     // Last value read from the button (HIGH = pressed)

// ---------------------------------------------------------------------------
//  L298 motor driver control pins
// ---------------------------------------------------------------------------
const int ENA = 10;  // PWM pin that sets the motor speed (0-255)
const int IN1 = 3;   // Direction pin A
const int IN2 = 4;   // Direction pin B

const int DEFAULT_SPEED = 120;   // Normal driving speed (PWM value)
const int TURNING_SPEED = 80;    // Slower speed intended for turns (currently not used)

// ---------------------------------------------------------------------------
//  Race state variables
// ---------------------------------------------------------------------------
char  side        = ' ';   // Turning direction of the track: 'l' = left, 'r' = right, ' ' = unknown yet
short corners     = 0;     // Number of corners completed (12 corners = 3 laps)
int   actual_Dist = 0;     // Current distance to the side wall (mm)
int   prev_Dist   = 0;     // Previous distance to the side wall (mm)
bool  run         = false; // true once the start button has been pressed
char round_type   = ' ';   // Round type: 'f' = free round (no obstacles), 'n' = non-free round (obstacles)

// ---------------------------------------------------------------------------
//  PID configuration (wall following)
// ---------------------------------------------------------------------------
double setpoint = 185.0;   // Desired distance to the side wall in mm
double error;              // Global error variable (calculatePID() uses its own local one)

// ---------------------------------------------------------------------------
//  Distance-to-time calibration table
// ---------------------------------------------------------------------------
// Each pair means: "driving for CAL_TIME_MS milliseconds moved the robot
// CAL_DIST_MM millimeters" at the calibration speed (d_speed).
const float CAL_DIST_MM[] = {   0,  10, 100, 250, 450,  560,  900, 1050 };   // Measured distances (mm)
const float CAL_TIME_MS[] = {   0, 100, 250, 500, 750, 1000, 1250, 1500 };   // Driving time that produced each distance (ms)
const int   CAL_POINTS    = sizeof(CAL_DIST_MM) / sizeof(CAL_DIST_MM[0]);    // Number of calibration points (8)
const int   d_speed       = 150;   // Motor speed used during calibration (and by move_d())

// Slope (ms per mm) used for distances longer than the last calibration point.
// It is averaged over the last 3 segments so one noisy measurement doesn't dominate.
const float EXTRAP_MS_PER_MM =
    (CAL_TIME_MS[CAL_POINTS - 1] - CAL_TIME_MS[CAL_POINTS - 4]) /
    (CAL_DIST_MM[CAL_POINTS - 1] - CAL_DIST_MM[CAL_POINTS - 4]);

// ===========================================================================
//  distanceToTimeMs()
//  Converts a positive distance (mm) into the driving time (ms) needed to
//  travel it, using linear interpolation on the calibration table.
// ===========================================================================
unsigned long distanceToTimeMs(float abs_distance) {
  if (abs_distance <= 0) return 0;   // No distance = no movement

  const float last_d = CAL_DIST_MM[CAL_POINTS - 1];   // Largest calibrated distance
  const float last_t = CAL_TIME_MS[CAL_POINTS - 1];   // Time for the largest calibrated distance
  float t;                                            // Resulting time in ms

  if (abs_distance >= last_d) {
    // Beyond the table: extrapolate using the average slope of the last segments
    t = last_t + (abs_distance - last_d) * EXTRAP_MS_PER_MM;
  } else {
    // Find the table segment [i-1, i] that contains this distance
    int i = 1;
    while (i < CAL_POINTS - 1 && abs_distance > CAL_DIST_MM[i]) i++;

    float d0 = CAL_DIST_MM[i - 1], d1 = CAL_DIST_MM[i];   // Segment start/end distances
    float t0 = CAL_TIME_MS[i - 1], t1 = CAL_TIME_MS[i];   // Segment start/end times

    // Linear interpolation between the two points
    t = t0 + (abs_distance - d0) * (t1 - t0) / (d1 - d0);
  }
  return (unsigned long)(t + 0.5f);   // Round to the nearest millisecond
}

// ===========================================================================
//  move_d()
//  Drives a given distance in mm (positive or negative sign selects the
//  direction) by running the motor for the calibrated time. Blocking.
// ===========================================================================
void move_d(float distance_mm) {
  int direction = (distance_mm >= 0) ? -1 : 1;                       // Sign convention: positive distance -> negative motor speed
  unsigned long move_time_ms = distanceToTimeMs(fabs(distance_mm));  // How long to drive

  if (move_time_ms == 0) return;   // Nothing to do

  move(direction * d_speed);   // Start the motor at calibration speed
  delay(move_time_ms);         // Wait while the robot moves (blocking)
  stop();                      // Stop the motor
}

// ===========================================================================
//  stop()
//  Stops the drive motor: speed 0 and both direction pins LOW.
// ===========================================================================
void stop() {
  analogWrite(ENA, 0);       // PWM speed = 0
  digitalWrite(IN1, LOW);    // Release direction pin A
  digitalWrite(IN2, LOW);    // Release direction pin B
}

// ===========================================================================
//  move(speed)
//  Starts the motor at the given speed and keeps it running (non-blocking).
//  speed > 0 : IN1 HIGH / IN2 LOW
//  speed < 0 : IN1 LOW  / IN2 HIGH
//  speed = 0 : stop
// ===========================================================================
void move(int speed)
{
  Serial.print ("Move speed :");   // Debug output
  Serial.print (speed);
  if (speed > 0)
  {
    digitalWrite(IN1, HIGH);   // Direction for positive speed
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);    // Direction for negative speed
    digitalWrite(IN2, HIGH);
  } else {
    stop();                    // Speed 0 -> stop and exit
    return;
  }

  analogWrite(ENA, abs(speed));   // Apply the speed magnitude as PWM
}

// ===========================================================================
//  move(speed, duration_ms)
//  Overloaded version: moves at a speed for a fixed time, then stops.
//  + = forward, - = backward
//  NOTE: the direction pins here are the OPPOSITE of move(speed) above.
// ===========================================================================
void move(int speed, int duration_ms) {
  //speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(IN1, LOW);    // Forward
    digitalWrite(IN2, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN1, HIGH);   // Backward
    digitalWrite(IN2, LOW);
  } else {
    stop();
    return;
  }

  analogWrite(ENA, abs(speed));   // Apply speed
  delay(duration_ms);             // Keep moving for the requested time (blocking)
  stop();                         // Then stop
}

// ===========================================================================
//  dist()
//  Returns the distance (mm) read by one sensor: l, f or r.
// ===========================================================================
double dist (int space)
{
  return sensors[space].readRangeContinuousMillimeters();
}

// ===========================================================================
//  turn_d()
//  Moves the steering servo to an angle and waits 0.5 s for it to arrive.
// ===========================================================================
void turn_d(int val)
{
  myservo.write(val);   // Set steering angle
  delay(500);           // Give the servo time to reach the position
}

// ===========================================================================
//  turn()
//  Corner maneuver: steers into the corner, counter-steers to straighten,
//  then waits until the side wall is visible again.
// ===========================================================================
void turn()
{
  stop();                // Stop before starting the corner
  delay(100);
  corners++;             // Count this corner
  // Steer toward the turn direction (MIN_ANGLE for left, MAX_ANGLE for right)
  myservo.write(side=='l'? MIN_ANGLE: MAX_ANGLE);
  Serial.print("Corner :");
  Serial.println (corners);
  move(DEFAULT_SPEED);   // Drive through the corner
  delay(2500);           // Time spent turning (tuned by testing)
  // Counter-steer briefly to straighten the car
  myservo.write(side=='l'? MAX_ANGLE: MIN_ANGLE);
  delay (800);
  myservo.write(CENTER_ANGLE);   // Wheels straight
  // Keep driving until the side wall is closer than 500 mm (out of the corner)
  while (dist(side == 'l'? l: r) > 500)
    {
    delay (50);
    continue;
    }
  Serial.println ("OUT corner ");
}

// ===========================================================================
//  pre_side_def()
//  Runs at the start of the race. Decides the round type (free or
//  non-free) and the turning direction of the track (left or right).
// ===========================================================================
void pre_side_def()
{
  // First reading of all three sensors
  Ldist = dist(l);
  Rdist = dist(r);
  Fdist = dist(f);
  Serial.println(Fdist);
  // If something is very close in front (< 150 mm) -> non-free round, otherwise free round
  round_type = (Fdist < 150? 'n': 'f');
  side = ' ';
  if (round_type == 'n')
    {
      // Non-free round: decide direction from the left wall distance
      side = (Ldist < 150? 'r': 'l')  ;
    }
  else
    {
        side =' ';   // Free round: direction still unknown
    }
  // If direction is unknown, drive forward until a side opens up (corner found)
  while (side == ' ')
  {
   move(DEFAULT_SPEED);
   Ldist = dist(l);
   Rdist = dist(r);
   Fdist = dist(f);
   Serial.println("LRF");   // Debug: print left, right, front readings
   Serial.println(Ldist);
   Serial.println(Rdist);
   Serial.println(Fdist);
      // A reading over 1000 mm means there is no wall on that side -> that is the corner
      if ((Ldist > 1000) || (Rdist > 1000))
      {
            stop();
            side = (Ldist > 500? 'l': 'r');   // Open side = turning direction
            Serial.println(side);
      }
  }
}

// ===========================================================================
//  calculatePID()
//  PID controller for wall following. Returns the servo angle needed to keep
//  the robot at 'setpoint' mm from the wall on the given side.
//  (Currently not called: its use in free_round() is commented out.)
// ===========================================================================
int calculatePID(double current_distance, char side) {
    static double last_error = 0;   // Error from the previous call (kept between calls)
    static double integral = 0;     // Accumulated error (kept between calls)

    // Tuning constants (these need to be adjusted by testing)
    double Kp = 0.066;  // Proportional gain
    double Ki = 0.01;   // Integral gain
    double Kd = 0.05;   // Derivative gain

    double error = setpoint - current_distance;   // Positive = too close to the wall

    // 1. Proportional term: reacts to the current error
    double P_out = error * Kp;

    // 2. Integral term: reacts to accumulated error, capped to prevent "windup"
    integral += error;
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;
    double I_out = integral * Ki;

    // 3. Derivative term: reacts to how fast the error is changing
    double D_out = (error - last_error) * Kd;
    last_error = error;

    double total_correction = P_out + I_out + D_out;   // Combined correction

    // Apply the correction around the center angle; sign depends on the wall side
    int target_angle = CENTER_ANGLE;
    if (side == 'r') {
        target_angle += (int)total_correction;
    } else {
        target_angle -= (int)total_correction;
    }

    // Keep the angle inside the servo's safe range
    if (target_angle < MIN_ANGLE) target_angle = MIN_ANGLE;
    if (target_angle > MAX_ANGLE) target_angle = MAX_ANGLE;

    return target_angle;
}

// ===========================================================================
//  calculateP()
//  Simpler proportional-only controller for wall following (target 200 mm).
//  Returns a servo angle. (Currently not called anywhere.)
// ===========================================================================
int calculateP(double current_distance, char side) {
    // Local setpoint; tuning differs from the PID because it is not time-scaled
    const double setpoint = 200.0;

    double error = setpoint - current_distance;   // Positive = too close to the wall

    int target_angle = CENTER_ANGLE;
    // Correction proportional to the relative error, up to about ±15 degrees
    if (side == 'l')
      target_angle += (int)(error/setpoint * 15);
    else
      target_angle -= (int)(error/setpoint * 15);
    return constrain(target_angle, MIN_ANGLE, MAX_ANGLE);   // Keep inside servo limits
}

// ===========================================================================
//  free_round()
//  Open challenge (no obstacles): follows the outer wall and turns at each
//  corner until 12 corners (3 laps) are completed.
// ===========================================================================
void free_round()
{
  corners = 0;
  prev_Dist = (side == 'l') ? dist(l) : dist(r);   // Initial distance to the wall being followed
  while (corners < 12)                             // 12 corners = 3 laps
  {
    move(DEFAULT_SPEED);   // Keep driving

    actual_Dist = (side == 'l') ? dist(l) : dist(r);   // Distance to the followed wall

    // Wall disappeared (> 1000 mm) -> we reached a corner
    if (actual_Dist > 1000)
      {
        stop();
        turn();
      }

    if (side == 'l')
        {
          // Wall following on the LEFT side (bang-bang steering)
          actual_Dist = dist(l);
          while (actual_Dist <= 1000)   // Repeat while the wall is visible
          {
            actual_Dist = dist(l);
            if (actual_Dist < 150)          // Too close -> steer away from the wall
              myservo.write(MAX_ANGLE);
            else if (actual_Dist > 200)     // Too far -> steer toward the wall
              myservo.write(MIN_ANGLE);
            delay (150);                    // Hold the correction briefly
            myservo.write(CENTER_ANGLE);    // Straighten the wheels again
          }
        }
    else
        {
          // Wall following on the RIGHT side (mirror of the left logic)
          actual_Dist = dist(r);
          while (actual_Dist <= 1000)
          {
            actual_Dist = dist(r);
            if (actual_Dist < 150)          // Too close -> steer away
              myservo.write(MIN_ANGLE);
            else if (actual_Dist > 200)     // Too far -> steer toward the wall
              myservo.write(MAX_ANGLE);
            delay (150);
            myservo.write(CENTER_ANGLE);
          }
        // Alternative: normal PID control (disabled)
        /*int servoPos = calculatePID(actual_Dist, side);
        myservo.write(servoPos);
        */
        //Serial.println(servoPos);
      }
    prev_Dist = actual_Dist;   // Save reading for the next cycle
  }
  stop();   // Race finished
}

// ===========================================================================
//  move_out()
//  Pre-programmed (timed, "blind") sequence to drive out of the parking
//  slot at the start of the obstacle round. Different for each direction.
// ===========================================================================
void move_out()
{
  if (side == 'l')
    {
      turn_d (MIN_ANGLE);    // Steer to one side
      move_d(500.0);         // Drive 500 mm
      turn_d (MAX_ANGLE);    // Steer to the other side
      move_d(200.0);         // Drive 200 mm
      turn_d (CENTER_ANGLE); // Wheels straight
    }
  else
    {
      turn_d (MAX_ANGLE);    // Multi-point maneuver for the right direction
      move_d(100.0);
      turn_d (MIN_ANGLE);
      move_d(-100.0);        // Negative = opposite direction
      turn_d (MAX_ANGLE);
      move_d(-100.0);
      turn_d (MIN_ANGLE);
      move_d(4000.0);
    }
}

// ===========================================================================
//  move_in()
//  Pre-programmed (timed) sequence to park inside the parking slot at the
//  end of the obstacle round. Both branches are currently identical.
// ===========================================================================
void move_in()
{
  if (side == 'l')
    {
      move_d(240.0);         // Drive past the slot
      turn_d (MAX_ANGLE);
      move_d(-340.0);        // Reverse into the slot
      turn_d (MIN_ANGLE);
      move_d(-140.0);        // Straighten while reversing
      turn_d (MAX_ANGLE);
      move_d(140.0);         // Final adjustment
    }
  else
    {
      move_d(240.0);
      turn_d (MAX_ANGLE);
      move_d(-340.0);
      turn_d (MIN_ANGLE);
      move_d(-140.0);
      turn_d (MAX_ANGLE);
      move_d(140.0);
    }
}

// ===========================================================================
//  non_free_round()
//  Obstacle challenge: drives 12 corners while reading the Nicla Vision
//  camera over I2C. Corner detection works; the reactions to red/green
//  pillars, parking slot and crosswalk are still placeholders (TODO).
// ===========================================================================
void non_free_round()
{
  corners = 0;
  VisionPacket packet;          // Local packet (hides the global 'packet')
  bool packet_valid = false;    // true only if the packet arrived complete and passed the checksum

  while (corners < 12)
  {
    move(DEFAULT_SPEED);   // Keep driving

    // ---------------- Poll the Nicla vision packet ----------------
    uint8_t buf[PACKET_SIZE];   // Raw byte buffer
    // Ask the camera for PACKET_SIZE bytes; 'received' = bytes actually returned
    uint8_t received = Wire.requestFrom((uint8_t)NICLA_I2C_ADDR, (uint8_t)PACKET_SIZE);
    packet_valid = false;

    if (received == PACKET_SIZE) {
      for (uint8_t i = 0; i < PACKET_SIZE; i++) buf[i] = Wire.read();   // Read all bytes
      memcpy(&packet, buf, PACKET_SIZE);                                // Copy bytes into the struct

      // Recalculate the checksum: XOR of every byte except the last one
      uint8_t calc_checksum = 0;
      for (uint8_t i = 0; i < PACKET_SIZE - 1; i++) calc_checksum ^= buf[i];

      if (calc_checksum == packet.checksum) {
        packet_valid = true;   // Data is trustworthy
      } else {
        Serial.println(F("CHECKSUM MISMATCH — Nicla packet discarded this cycle."));
      }
    } else {
      // Camera did not return a full packet
      Serial.print(F("I2C read error — expected "));
      Serial.print(PACKET_SIZE);
      Serial.print(F(" bytes, got "));
      Serial.println(received);
    }

    // ---------------- Wall-following / corner detection ----------------
    Fdist = dist(f);
    if (Fdist < 600)   // Wall ahead closer than 600 mm -> corner
    {
      if (side != ' ')
        turn();        // Direction already known -> turn
      else
      {
        // Direction unknown -> find the open side first, then turn
        Ldist = dist(l);
        Rdist = dist(r);
        stop();
        side = (Ldist > 1000 ? 'l' : 'r');
        turn();
      }
    }

    // ---------------- Vision-driven branches — independent checks, not else-if ----------------
    // NOTE: priority/exclusivity between simultaneous flags is UNRESOLVED.
    // If red+green+yellow all fire in the same packet, all four blocks below
    // execute in this same iteration. Fine for stub comments; will conflict
    // the moment any block issues real servo/motor commands. Decide priority
    // before implementing actual maneuvers here.

    if (packet_valid && (packet.status_flags & FLAG_RED)) {
      // RED LOGIC:
      // packet.red_cx (0-159, QQVGA frame width), packet.red_distance_mm available.
      // Rule 9.19: red pillar must be passed on its RIGHT side.
      // TODO: steer away from cx toward the correct side once avoidance
      // strategy is finalized (see unresolved scope question — cx-based
      // proportional steering vs. current width-comparison heuristic).
    }

    if (packet_valid && (packet.status_flags & FLAG_GREEN)) {
      // GREEN LOGIC:
      // packet.green_cx, packet.green_distance_mm available.
      // Rule 9.19: green pillar must be passed on its LEFT side.
      // TODO: same open decision as RED LOGIC above.
    }

    if (packet_valid && (packet.status_flags & FLAG_PARKING)) {
      // MAGENTA LOGIC (parking slot):
      // packet.parking_cx available. packet.parking_distance_mm may be
      // NOT_FOUND_SENTINEL (0xFFFF) even when found=true — this happens in
      // "gap" mode (both delimiters visible separately, no single known-width
      // reference). Check for the sentinel before using distance here.
      // TODO: this is currently NOT wired into move_in()/move_out(), which
      // still run blind timed dead-reckoning sequences. Real parking-slot
      // targeting using parking_cx is a separate, larger change — flagged
      // previously as a scoring risk against rule 1.8.2 (15 pts for full park).
    }

    if (packet_valid && (packet.status_flags & FLAG_YELLOW)) {
      // STRIPES LOGIC (crosswalk / surprise rule):
      // packet.yellow_closest_mm, packet.yellow_farthest_mm available.
      // TODO: required ACTION on detecting the crosswalk is still UNKNOWN —
      // the WRO surprise-rule teaser's mission text is redacted pending the
      // official on-day reveal. This block intentionally does nothing yet.
      // Do not guess an action here (stop/slow/yield) — wire it in only
      // once the rule is officially announced.
      Serial.println(F("Crosswalk detected — action pending surprise rule reveal."));
    }

    delay(150);  // Loop polling interval (a guess) — tune once the real I2C
                 // throughput with the ToF sensors + Nicla sharing the bus is measured.
  }

  // Parking maneuver entry point — existing code, unchanged
}

// ===========================================================================
//  setup()
//  Runs once at power-on: serial port, servo, button, motor driver, I2C and
//  the three distance sensors (each gets its own I2C address).
// ===========================================================================
void setup()
{
  Serial.begin (9600);   // Serial monitor for debugging

  myservo.attach(8);            // Steering servo on pin 8
  myservo.write(CENTER_ANGLE);  // Start with the wheels straight

  delay(1000);   // Wait one second for the servo to settle

  pinMode(keyPin,INPUT);    // Start button as input
  pinMode(keyLed, OUTPUT);  // Status LED as output

  pinMode(ENA, OUTPUT);     // Motor driver pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  stop();                   // Make sure the motor is stopped

  Wire.begin();   // Start the I2C bus
  short s_count = sizeof(sensors) / sizeof(sensors[0]);   // Number of sensors (3)

  // Step 1: put ALL sensors in reset so they don't answer on the shared default address
  for (int i = 0; i < s_count; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);   // LOW on XSHUT = sensor off/reset
  }

  // Step 2: wake the sensors one at a time and give each a unique I2C address
  for (int i = 0; i < s_count ; i++) {
    digitalWrite(xshutPins[i], HIGH);  // Bring this sensor out of reset
    delay(10);                         // Allow time for it to boot

    sensors[i].init();                     // Initialize the sensor
    sensors[i].setTimeout(500);            // Give up a reading after 500 ms
    sensors[i].setAddress(addresses[i]);   // Assign its unique address
  }

  // Step 3: start continuous measuring mode on every sensor
  for (int i = 0; i < s_count; i++) {
  sensors[i].startContinuous();
  }
  //run = true;   // Uncomment to skip the start button during testing
}

// ===========================================================================
//  loop()
//  Main program: waits for the start button, detects round type and
//  direction, runs the matching round, then resets and waits again.
// ===========================================================================
void loop()
{
  stop();                       // Motor off while waiting
  digitalWrite(keyLed, LOW);    // LED off = not started
  while (!run)                  // Wait for the start button
  {
    buttonState = digitalRead(keyPin);
    if(buttonState == HIGH )
      {
        run = true;                  // Button pressed -> start
        digitalWrite(keyLed, HIGH);  // LED on to confirm
        delay(1000);                 // Short pause before moving
        break;
      }
    else
      run = false;

    Serial.write("\nWaiting button...");
    delay(100);
  }
  /////////////////
  // Test area after the button press (for quick experiments)

  ///////////////
    stop();
    // Sensor debug loop — disabled with while(false); change to while(true) to print readings every second
    while (false)
    {
    Ldist = dist(l);
    Rdist = dist(r);
    Fdist = dist(f);
    Serial.println("LRF");
    Serial.println(Ldist);
    Serial.println(Rdist);
    Serial.println(Fdist);
    delay(1000);
    }

    pre_side_def();               // Decide round type and turning direction
  Serial.println (round_type) ;
    if (round_type == 'n' )
      {
      non_free_round();           // Obstacle challenge
      }
    else if (round_type == 'f')
      {
      free_round();               // Open challenge
      }
    run = false;                  // Reset so the robot waits for the button again
    buttonState = 0;
}   // End of loop() (this closing brace was missing in the original code)

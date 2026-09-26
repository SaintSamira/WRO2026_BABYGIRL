# WRO Future Engineers 2026 — US Open
Autonomous self-driving vehicle engineering repository for the World Robot Olympiad Future Engineers category.
- Team: Bianca Polato & Samira Santos 
- Coach: Julian Vigil School: Howard Academy (Panama) 
- Event: WRO Future Engineers — US Open 2026
---

## Table of Contents
1. Team Introduction
2. The Challenge
3. Repository Structure
4. Mechanical Design
5. Electronics & Power Distribution
6. Software & Navigation Logic


## Team Introduction
Our team is composed of two members, working under the guidance of our coach, Julian Vigil.

- Bianca Polato - Team Captain & Mechanical Lead: Responsible for the physical build of the robot, including chassis assembly, mechanical integration, and hands-on electronics work such as soldering and wiring connections.
- Samira Santos: - Software Lead & Electronics Planning: Responsible for programming the robot's control logic and computer vision, as well as planning the electronics architecture that Bianca implements physically.

This is our third year competing in WRO Future Engineers, and our first time qualifying for the Open. This program has always been, for us, about the process of learning: refining engineering reasoning, testing rigorously, and improving through iteration, rather than solely about the outcome of winning

## The Challenge
WRO Future Engineers challenges teams to design, build, and program a fully autonomous vehicle capable of navigating a reconfigurable race track without any human intervention or remote control once a round begins. The competition includes:
- **Open Challenge:** the vehicle must complete three consecutive laps around a track with a variable-size inner section, staying within the track boundaries.
- **Obstacle Challenge:** in addition to completing laps, the vehicle must identify and correctly respond to red and green traffic pillars along the track, and execute a parallel parking maneuver.
- **Suprise Challenge:** finally there was a recent addition to the rules. We are not sure on how what it is, but what we do know is that it involes a pedestrian cross walk.

Our current development priority is achieving a reliable, complete lap cycle in the Open Challenge before moving on to obstacle detection and parking.

## Repositiry Structure
- Model: includes all the digital models of the 3D printed bases we used.
- Schemes: includes a wiring diagrm of the electricl components of our vehicle.
- Src: includes the code we used for the detection and movement of the component in our vehicle
- t-photos: includes 2 fotos of the team.
- v-photos: includes fotos of the vehicle for different points of views.
- video: includes the car doing one lap.

## Mechanical Design
### Chassis
The V2.0 chassis is built on a modified metal car-building base kit (Olibots Arduino Advanced), which has been extensively adapted from its original configuration. This represents a complete departure from the LEGO SPIKE-based construction used in our earlier versions (V1.0 and V1.5), transitioning the team fully to open-source hardware and metal components.The wheels, steering servo, and L298N H-bridge motor driver were included with the Olibots kit. The metal axle and gears were reused from the school’s VEX V5 program.

The main structure consists of three metal plates connected by metal spacers. The middle plate was extensively modified: its rear section was cut out to create space for the DC motor mounted on the lower plate, while a central hole was added to accommodate the steering servo and allow it to connect to the front steering system. The lower plate was also modified by adding a central opening that provides access to the steering servo, allowing adjustments to be made without disassembling the entire chassis. Additional mounting holes were drilled into both all the plates to secure the L298N motor driver, I2C bus, Arduino board, servo motor and motor mount. Recently we added a four plate to add weight into the frontal area of the car.

### Steering System 
The vehicle uses an Ackermann-inspired front steering geometry, in which both front wheels turn at slightly different angles through the servo-actuated linkage, allowing smoother and more mechanically accurate cornering than a simple pivot-steering design.

### Drivetrain
Drive is provided by a Bringsmart JGB37-520 12V 30RPM DC gear motor, connected through a set of VEX V5 gears and a metal shaft to the rear axle. The L298N dual H-bridge module handles motor direction and speed control.

### Sensor Mounting
Two VL53L0X Time-of-Flight (ToF) laser distance sensors are mounted diagonally in a 3D-printed housing at the front of the vehicle. This housing was reused and adapted from a 3D print made in an earlier WRO season. Mounting the sensors diagonally at the front (rather than symmetrically on the sides, as in previous versions) reduces mechanical interference with the steering system while preserving detection range.

## Code Logic Explanation

The navigation code is written in C++ for the Arduino platform. It uses three VL53L0X time-of-flight distance sensors (left, front, right), a steering servo, a DC drive motor, and I2C communication with a NICLA Vision camera module that handles color detection.

### Hardware setup
- **Distance sensors:** The three VL53L0X sensors share the I2C bus. All VL53L0X units start with the same default address, so at startup every sensor is held in reset through its XSHUT pin (pins 7, 6, 5). The sensors are then woken one at a time and each is assigned a unique I2C address (0x30, 0x31, 0x32). After that, all three start continuous ranging.
- **Drive motor:** A single DC motor is controlled through an L298N H-bridge. ENA sets the speed with PWM, and IN1/IN2 set the direction. The normal driving speed is `DEFAULT_SPEED = 120`.
- Steering servo: The servo is on pin 8. It has a center position `(CENTER_ANGLE = 83)` and a safe range `(MIN_ANGLE = 48, MAX_ANGLE = 118)` to prevent over-steering.
- Start button and LED: A push button (`keyPin`, A3) starts the run, and an LED (`keyLed`, pin 13) turns on to confirm the button was pressed.

### Main loop flow
1. The robot stops the motor and waits for the start button.
2. `pre_side_def()` runs once at the start of the round. It decides both the round type and the turning direction of the track.
    - If the front sensor reads less than 150 mm (the robot starts facing a close wall, as in the parking-slot start), the round is treated as the Obstacle Challenge (`round_type = 'n'`). The direction is taken from the left wall: if it is closer than 150 mm the track turns right, otherwise it turns left.
    - Otherwise, the round is treated as the Open Challenge (`round_type = 'f'`). The robot drives forward while reading the left and right sensors. The first side whose distance exceeds 1000 mm (meaning that wall has ended) sets the turning direction, which is stored in `side` for the rest of the round.
3. Depending on `round_type`, the robot runs either `free_round()` (Open Challenge, no obstacles) or `non_free_round()` (Obstacle Challenge, with pillars and parking).
4. When the round ends, the robot resets and waits for the start button again.

### Corner detection and turning (`turn()`)
A corner is detected differently in each round:
- In the Open Challenge, it is detected when the side sensor in use reads more than 1000 mm (the side wall has ended).
- In the Obstacle Challenge, it is detected when the front sensor reads less than 600 mm (a wall is ahead).

Once a corner is detected, `turn()` executes a fixed maneuver:
1. It stops, increments the corner counter, and steers fully toward the turning direction (`MIN_ANGLE` for left turns, `MAX_ANGLE` for right turns).
2. It drives forward for 2.5 seconds.
3.It counter-steers for 0.8 seconds to straighten the car, then returns the servo to center.
4. It keeps driving until the relevant side sensor detects a wall again (distance under 500 mm), which confirms the turn is complete.

Wall-following (`free_round()`)
Between corners, the robot follows the side wall on the turning side using threshold-based steering:
- If the distance to the wall drops below 150 mm, it steers away from the wall.
- If the distance exceeds 200 mm, it steers back toward the wall.
- Each correction is held for 150 ms, and then the servo returns to center.
This repeats until 12 corners have been completed (3 laps of 4 corners each), which ends the round.

> Note: The code also includes calculatePID() and calculateP(). These are a proportional-integral-derivative controller (target 185 mm) and a simple proportional wall-following controller (target 200 mm), written as alternatives to the threshold-based steering that is currently active. They are in the code but not yet wired into the control loop. They are a planned refinement for smoother, more precise wall-following once the current logic is validated.

### Distance-based movement (`move_d()`)
For precise, short maneuvers such as parking, the robot uses dead reckoning: it moves a distance by running the motor for a calibrated amount of time.
- A calibration table (`CAL_DIST_MM` / `CAL_TIME_MS`) was built by measuring how far the robot traveled for different driving times at a fixed speed (`d_speed = 150`).
- `distanceToTimeMs()` converts a requested distance into a driving time by linear interpolation between the two nearest table points.
- For distances beyond the last calibrated point (1050 mm), it extrapolates using the average slope of the last three segments, so a single noisy measurement doesn't dominate.
- `move_d()` then runs the motor for that time and stops. A positive or negative distance selects the direction.

## Sensor & Vision Processing (NICLA Vision, Python/OpenMV)
The distance sensors, steering, and drive logic run on the Arduino. Obstacle and marker detection is handled separately by the NICLA Vision board, which runs MicroPython through the OpenMV IDE (`main.py`). The NICLA acts as an I2C slave at address 8 and continuously prepares an 18-byte data packet that the Arduino requests each cycle (see Object Avoidance Logic below).

## Run modes
The NICLA code has two modes, selected with `RUN_MODE` before flashing:
- `STANDALONE` runs detection and prints results to the serial monitor only, without using I2C. It is used for bench calibration and testing.
- `SLAVE` runs detection and sends the results to the Arduino over I2C. This mode must be active during competition runs.
  
## Camera setup
- The camera runs at a low resolution (QQVGA, 160×120 px) in RGB565 color format. This keeps the frame rate high enough for real-time driving decisions.
- Auto gain and auto white balance are disabled so color readings stay consistent as lighting changes. This is essential for reliable color-threshold detection.
- Only the bottom half of the frame is analyzed, since that is where nearby track objects appear. The top half, above the horizon, contains background clutter and is ignored. The bottom half is split into two Regions of Interest (ROIs):
  - Pillar ROI (upper part of the bottom half) is for upright objects: pillars and parking-slot walls.
  - Ground ROI (bottom strip) is for markings on the floor: crosswalk stripes.

## Color detection
All colors are detected with LAB color thresholds, tuned with OpenMV's built-in Threshold Editor. The values currently in the code are placeholders and must be recalibrated for the team's actual objects and the competition lighting before testing.
- **Red and green pillars (`find_pillars()`):** For each color, the code finds all matching blobs in the pillar ROI and keeps only the largest one, assuming it is the closest pillar of that color. Both colors are reported independently, so the Arduino can receive a red and a green pillar in the same frame.
- **Magenta parking slot (`find_magenta()`):** This works in one of two modes.
In gap mode, both parking walls are visible as separate blobs, and the target is the midpoint between them.
In single-blob mode, only one wall is visible, and its width is used to estimate distance.
- **Yellow crosswalk stripes (`find_crosswalk()`):** Yellow blobs in the ground ROI are first filtered by shape (their width-to-height ratio must look like a stripe). A crosswalk is only confirmed if at least 3 stripes are found and they are spaced at fairly regular intervals. This rejects random yellow noise.

## Distance estimation
The distance to each detected object is estimated with the pinhole-camera relationship:
> distance = (real object width × focal length in pixels) / apparent width in pixels

The known real sizes are:
- pillar width: 50 mm
- parking wall: 200 mm
- stripe thickness: 15 mm
The focal length is `FOCAL_LENGTH_PX = 157`. These values must match the team's actual objects and camera setup. In gap mode the parking slot has no single known-width reference, so no distance is sent for it.

## Sending data to the Arduino
Each frame, the detection results are packed into a fixed 18-byte binary packet with struct.pack('<BHHHHHHHH', ...), plus a checksum byte. It uses the same little-endian layout as the VisionPacket struct on the Arduino side, so the two boards exchange data directly without extra parsing.

Any missing value is sent as 0xFFFF. The I2C slave buffer is refreshed with new data every frame, so whenever the Arduino requests data it receives the most recently processed frame's results.

## Parking Logic (Parallel Parking)
The code already contains two parking routines built on the distance-based movement described above:
- `move_out()` drives the robot out of the parking slot at the start of the Obstacle Challenge. It uses a sequence of steering angles and timed moves that differs depending on the turning direction.
- `move_in()` parks the robot at the end of the round. It drives past the slot, reverses into it at an angle, straightens while reversing, and makes a final forward adjustment.
Both routines are currently "blind": they follow fixed timed sequences and do not yet use the camera's parking_cx data. They are also not called from the main loop yet. The point where the 12th corner is completed in non_free_round() is marked as the entry point for parking.

## Object Avoidance Logic
The Obstacle Challenge round (`non_free_round()`) uses the same corner-turning logic as the Open Challenge, and adds data from the NICLA Vision module.

**How it works**
1. **Request data:** On each control cycle (about every 150 ms), the Arduino requests the 18-byte packet from the NICLA at I2C address 8.
2. **Validate:** The Arduino recalculates the XOR checksum. If it doesn't match, or if fewer than 18 bytes arrive, the packet is discarded for that cycle and an error is printed to the serial monitor. This prevents corrupted data from causing bad steering decisions.
3. **Corners:** If the front sensor reads less than 600 mm, the robot treats this as approaching a corner and hands off to `turn()`.
4. **Vision flags:** The robot then checks each flag in the packet independently:
     - **Red pillar:** The official WRO rule requires red pillars to be passed on their right side. The position (`red_cx`) and distance are available for steering.
      - **Green pillar:** The official WRO rule requires green pillars to be passed on their left side. The position (`green_cx`) and distance are available for steering.
      - **Parking slot:** Its position is available for aligning the parking maneuver.
      - **Crosswalk:** The required action depends on the WRO surprise rule, which has not been officially revealed yet. The robot only logs the detection for now.

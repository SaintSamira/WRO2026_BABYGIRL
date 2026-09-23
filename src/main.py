# ============== NICLA VISION - PYTHON ==============
# Save as main.py on the Nicla Vision (OpenMV IDE).
#
# MODE_STANDALONE: runs detection + Serial print only, no I2C. Use this for
#                   bench calibration and isolated testing before wiring to Mega.
# MODE_SLAVE:       runs detection + I2C slave buffer updates for the Mega to poll.
#
# No ToF/distance-sensor code exists or has ever existed in this file —
# all distance estimates are vision-based (known-width-to-pixel-width formula).

import sensor
import image
import time
from pyb import I2C
import struct

# =========================================================================
# MODE FLAG — set before flashing.
# =========================================================================
MODE_STANDALONE = "STANDALONE"
MODE_SLAVE      = "SLAVE"
RUN_MODE = MODE_STANDALONE   # <-- flip to MODE_SLAVE only after standalone testing passes clean

I2C_SLAVE_ADDR = 8  # must match Mega's I2C_SLAVE_ADDRESS

print("RUN_MODE active:", RUN_MODE)  # sanity check — must appear immediately on boot

# =========================================================================
# Camera Configuration
# =========================================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

FRAME_W = sensor.width()
FRAME_H = sensor.height()

# =========================================================================
# CALIBRATION REQUIRED — recalibrate via OpenMV IDE Threshold Editor
# =========================================================================
GREEN_THRESHOLD   = (30, 80, -70, -30, 0, 70)   # [Guessing] placeholder, from your file
RED_THRESHOLD     = (30, 80, 30, 80, 10, 70)     # [Guessing] placeholder, from your file
MAGENTA_THRESHOLD = (20, 80, 30, 80, -60, -10)   # [Guessing] carried over — recalibrate at QQVGA
YELLOW_THRESHOLD  = (60, 100, -10, 30, 30, 90)   # [Guessing] no official spec published yet

FOCAL_LENGTH_PX = 157.0  # [Likely] your prior calibrated value — preserved, do not overwrite blindly

# =========================================================================
# KNOWN GEOMETRY (mm) — from WRO 2026 ruleset where cited, else placeholder
# =========================================================================
PILLAR_WIDTH_MM   = 50    # rule 13.1 — confirmed
DELIMITER_LONG_MM = 200   # rule 13.7 — assumes long face visible to camera [Guessing]
STRIPE_WIDTH_MM   = 15    # 1.5 cm — short edge/thickness [Guessing on axis mapping]
STRIPE_LENGTH_MM  = 100   # 10 cm  — long edge/span       [Guessing on axis mapping]
STRIPE_ASPECT_RATIO = STRIPE_LENGTH_MM / STRIPE_WIDTH_MM  # ~6.7
STRIPE_ASPECT_TOLERANCE = 0.5  # [Guessing] wide net — tighten once real footage exists
MIN_STRIPE_COUNT = 3  # [Guessing]

def estimate_distance(pixel_dim, real_dim_mm, focal_px=FOCAL_LENGTH_PX):
    if pixel_dim <= 0:
        return -1
    return int((real_dim_mm * focal_px) / pixel_dim)

# =========================================================================
# ROIs — computed from actual frame size, not hardcoded resolution
# =========================================================================
band_top = FRAME_H // 2
pillar_roi_h = (FRAME_H - band_top) * 2 // 3
pillar_roi = (0, band_top, FRAME_W, pillar_roi_h)                              # upright objects
ground_roi = (0, band_top + pillar_roi_h, FRAME_W, FRAME_H - band_top - pillar_roi_h)  # ground-plane stripes

# =========================================================================
# Detection functions
# =========================================================================
def find_pillars(img):
    results = []
    for thresh, name in [(RED_THRESHOLD, "RED"), (GREEN_THRESHOLD, "GREEN")]:
        blobs = img.find_blobs([thresh], roi=pillar_roi, pixels_threshold=60,
                                area_threshold=60, merge=True)
        if blobs:
            b = max(blobs, key=lambda x: x.pixels())
            results.append({"id": name, "cx": b.cx(), "w": b.w(),
                             "distance_mm": estimate_distance(b.w(), PILLAR_WIDTH_MM),
                             "blob": b})
    return results

def find_magenta(img):
    blobs = img.find_blobs([MAGENTA_THRESHOLD], roi=pillar_roi, pixels_threshold=30,
                            area_threshold=30, merge=False)
    if not blobs:
        return None
    if len(blobs) >= 2:
        top2 = sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:2]
        left, right = sorted(top2, key=lambda b: b.cx())
        return {"cx": (left.cx() + right.cx()) // 2, "mode": "gap",
                "left": left, "right": right}
    b = blobs[0]
    return {"cx": b.cx(), "mode": "single_blob", "w": b.w(), "blob": b}

def _is_stripe_shaped(b):
    if b.h() <= 0:
        return False
    ratio = b.w() / b.h()
    lo = STRIPE_ASPECT_RATIO * (1 - STRIPE_ASPECT_TOLERANCE)
    hi = STRIPE_ASPECT_RATIO * (1 + STRIPE_ASPECT_TOLERANCE)
    return lo <= ratio <= hi

def find_crosswalk(img):
    raw_blobs = img.find_blobs([YELLOW_THRESHOLD], roi=ground_roi, pixels_threshold=15,
                                area_threshold=15, merge=False)
    blobs = [b for b in raw_blobs if _is_stripe_shaped(b)]
    if len(blobs) < MIN_STRIPE_COUNT:
        return {"detected": False, "blobs": []}
    blobs = sorted(blobs, key=lambda b: b.cx())
    gaps = [blobs[i+1].cx() - blobs[i].cx() for i in range(len(blobs) - 1)]
    avg_gap = sum(gaps) / len(gaps)
    gap_var = sum(abs(g - avg_gap) for g in gaps) / len(gaps)
    if gap_var >= (avg_gap * 0.6):
        return {"detected": False, "blobs": []}
    return {"detected": True, "blobs": blobs}

def build_detection_report(pillars, magenta, crosswalk):
    report = {
        "red_pillar":    {"found": False, "cx": None, "distance_mm": None},
        "green_pillar":  {"found": False, "cx": None, "distance_mm": None},
        "parking_slot":  {"found": False, "cx": None, "distance_mm": None},
        "yellow_stripes":{"found": False, "closest_mm": None, "farthest_mm": None}
    }
    for p in pillars:
        key = "red_pillar" if p["id"] == "RED" else "green_pillar"
        report[key] = {"found": True, "cx": p["cx"], "distance_mm": p["distance_mm"]}

    if magenta:
        dist = estimate_distance(magenta["w"], DELIMITER_LONG_MM) if magenta["mode"] == "single_blob" else None
        report["parking_slot"] = {"found": True, "cx": magenta["cx"], "distance_mm": dist}

    if crosswalk["detected"]:
        distances = [estimate_distance(b.h(), STRIPE_WIDTH_MM) for b in crosswalk["blobs"]]
        distances = [d for d in distances if d > 0]
        if distances:
            report["yellow_stripes"] = {"found": True, "closest_mm": min(distances),
                                         "farthest_mm": max(distances)}
    return report

# =========================================================================
# I2C Packet — single canonical 18-byte struct
# =========================================================================
NOT_FOUND_SENTINEL = 0xFFFF
FLAG_RED, FLAG_GREEN, FLAG_PARKING, FLAG_YELLOW = 0x01, 0x02, 0x04, 0x08

def build_i2c_packet(detection):
    flags = 0
    if detection["red_pillar"]["found"]:    flags |= FLAG_RED
    if detection["green_pillar"]["found"]:  flags |= FLAG_GREEN
    if detection["parking_slot"]["found"]:  flags |= FLAG_PARKING
    if detection["yellow_stripes"]["found"]: flags |= FLAG_YELLOW

    def safe(v):
        return v if v not in (None, -1) else NOT_FOUND_SENTINEL

    red, green = detection["red_pillar"], detection["green_pillar"]
    park, yel = detection["parking_slot"], detection["yellow_stripes"]

    payload = struct.pack('<BHHHHHHHH', flags,
        red["cx"] or 0, safe(red["distance_mm"]),
        green["cx"] or 0, safe(green["distance_mm"]),
        park["cx"] or 0, safe(park["distance_mm"]),
        safe(yel["closest_mm"]), safe(yel["farthest_mm"]))

    checksum = 0
    for b in payload:
        checksum ^= b
    return payload + bytes([checksum])  # 18 bytes total

# =========================================================================
# I2C Slave Init — only in MODE_SLAVE
# =========================================================================
i2c = None
if RUN_MODE == MODE_SLAVE:
    packed_data = bytes(18)
    i2c = I2C(1, I2C.SLAVE, addr=I2C_SLAVE_ADDR)
    i2c.init(I2C.SLAVE, addr=I2C_SLAVE_ADDR, g_slave_buf=packed_data)
    print("I2C slave initialized at addr:", I2C_SLAVE_ADDR)

print("Nicla Vision Ready.")
clock = time.clock()

# =========================================================================
# Main Loop
# =========================================================================
while True:
    clock.tick()
    img = sensor.snapshot()
    img.draw_rectangle(pillar_roi, color=(40, 40, 40))
    img.draw_rectangle(ground_roi, color=(60, 60, 20))

    pillars   = find_pillars(img)
    magenta   = find_magenta(img)
    crosswalk = find_crosswalk(img)

    for p in pillars:
        c = (255, 0, 0) if p["id"] == "RED" else (0, 255, 0)
        img.draw_rectangle(p["blob"].rect(), color=c)

    if magenta:
        if magenta["mode"] == "gap":
            img.draw_rectangle(magenta["left"].rect(), color=(255, 0, 255))
            img.draw_rectangle(magenta["right"].rect(), color=(255, 0, 255))
        else:
            img.draw_rectangle(magenta["blob"].rect(), color=(255, 0, 255))

    if crosswalk["detected"]:
        for b in crosswalk["blobs"]:
            img.draw_rectangle(b.rect(), color=(255, 255, 0))

    detection = build_detection_report(pillars, magenta, crosswalk)

    if RUN_MODE == MODE_SLAVE:
        packed_data = build_i2c_packet(detection)
        # RISK, UNRESOLVED: deinit/init every frame to refresh g_slave_buf leaves
        # a window where a master read can hit a torn/absent buffer. This is
        # your original file's pattern, preserved as-is rather than silently
        # replaced. If reliability issues show up as intermittent Mega checksum
        # failures, this is the first suspect — not the checksum logic itself.
        i2c.deinit()
        i2c.init(I2C.SLAVE, addr=I2C_SLAVE_ADDR, g_slave_buf=packed_data)
    else:  # MODE_STANDALONE
        print("FPS:%.1f" % clock.fps(), detection

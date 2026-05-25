"""Numeric constants for the prosthetic-hand pipeline.

Single source of truth. The Arduino firmware (motor_control.ino,
sensor_stream.ino) hold their own copies of the motor / sampling
constants; if any of these change here, update the matching #define
or const in the .ino file by hand.
"""

# ----------------------------------------------------------------
# Motor geometry  (must match motor_control.ino)
# ----------------------------------------------------------------
OPEN_POS_DEG     = 66.0
CLOSE_POS_DEG    = 110.0
GRASP_SPEED_RPM  = 10.0
CALIB_SPEED_RPM  = 1.0   # see motor_control.ino comment: slowed from 3.0
                         # to accommodate the XIAO's ~7-10 Hz sensor rate.

# ----------------------------------------------------------------
# Sampling
# ----------------------------------------------------------------
SAMPLE_INTERVAL_MS = 20         # 50 Hz on both XIAO and OpenRB
SAMPLE_HZ          = 1000 // SAMPLE_INTERVAL_MS

# ----------------------------------------------------------------
# Motor-field profile
# ----------------------------------------------------------------
PROFILE_BIN_DEG_MIN = int(OPEN_POS_DEG)          # 66
PROFILE_BIN_DEG_MAX = int(CLOSE_POS_DEG)         # 110
PROFILE_NUM_BINS    = PROFILE_BIN_DEG_MAX - PROFILE_BIN_DEG_MIN + 1  # 45

# ----------------------------------------------------------------
# Contact / grasp detection  (mirrored on OpenRB for stall path)
# ----------------------------------------------------------------
CONTACT_IGNORE_MS        = 150
# CONTACT_CONFIRM_SAMPLES: number of consecutive over-threshold sensor
# samples needed to fire magnetic t1. The XIAO at gain=0, osr=2, df=5
# runs at ~7-10 Hz (each readData is ~25 ms x 4 sensors). At this rate,
# 8 samples would mean ~1.1 s of sustained signal -- so long that the
# OpenRB's encoder-stall path (~15 ms) always wins, even for stiff
# objects that should fire magnetic first. Two samples gives ~200-280
# ms which is competitive with stall while staying robust to noise
# (false-positive probability at 1.8x-p99 thresholds is ~0.01% per pair).
CONTACT_CONFIRM_SAMPLES  = 5
ENCODER_STILL_DEG        = 0.5
ENCODER_STILL_COUNT      = 5
ENCODER_STALL_DEG        = 0.3
ENCODER_STALL_CONFIRM    = 4
ENCODER_MOVING_MIN_DEG   = 1.0
HOLD_DURATION_MS         = 800
GRASP_TIMEOUT_MS         = 5000

# ----------------------------------------------------------------
# Magnetic-threshold defaults (overridden per session by auto-derive)
# Order: per sensor, [x_threshold, y_threshold, z_threshold]
# ----------------------------------------------------------------
DEFAULT_CONTACT_THRESHOLDS = [
    [40.0, 40.0, 65.0],   # s0 Index ID1
    [40.0, 40.0, 60.0],   # s1 Index ID2
    [25.0, 25.0, 35.0],   # s2 Thumb ID3
    [25.0, 25.0, 30.0],   # s3 Thumb ID4 (most sensitive)
]
# Auto-derivation rule:
#   threshold[s][a] = max(THRESHOLD_FLOOR,
#                         noise_p99[s][a] * THRESHOLD_SAFETY_FACTOR)
THRESHOLD_FLOOR          = 1.0
THRESHOLD_SAFETY_FACTOR  = 1.8

# ----------------------------------------------------------------
# Sensors
# ----------------------------------------------------------------
NUM_SENSORS   = 4
NUM_AXES      = 3
NUM_CHANNELS  = NUM_SENSORS * NUM_AXES   # 12
SENSOR_ADDRS  = [0x14, 0x15, 0x16, 0x17]
SENSOR_NAMES  = ['s0', 's1', 's2', 's3']

# ----------------------------------------------------------------
# Features
# ----------------------------------------------------------------
NUM_FEATURES  = 57

# ----------------------------------------------------------------
# Classes (alphabetical so sklearn LabelEncoder gives stable order)
# ----------------------------------------------------------------
CLASS_NAMES   = ['cube_soft', 'cube_stiff', 'cylinder_soft', 'cylinder_stiff', 'no_object']
LABEL_TO_KEY  = {str(i + 1): name for i, name in enumerate(CLASS_NAMES)}
UNKNOWN_CLASS = 'unknown_object'

# ----------------------------------------------------------------
# Serial / USB
# ----------------------------------------------------------------
SERIAL_BAUD       = 115200
SERIAL_TIMEOUT_S  = 0.2

# Auto-detect VID/PID hints (any match acceptable; we still ping to
# verify which board is which after open).
XIAO_USB_HINTS   = [
    (0x303A, None),  # Espressif Systems (covers XIAO ESP32S3)
]
OPENRB_USB_HINTS = [
    (0x2E8A, None),  # Raspberry Pi (OpenRB-150 uses RP2040 bridge)
    (0x2B62, None),  # ROBOTIS
]

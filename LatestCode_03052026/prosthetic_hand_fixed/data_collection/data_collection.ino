/*
 * ================================================================
 *  PROSTHETIC HAND - DATA COLLECTION FIRMWARE  (4-sensor, amplified)
 *  File: data_collection.ino
 * ================================================================
 *
 *  Purpose:
 *    Collect labelled single-grasp tactile data for object recognition
 *    using INDEX + THUMB sensors only. Middle finger has been removed
 *    from the firmware after persistent hardware failure on TCA ch 2.
 *
 *  Hardware layout:
 *    OpenRB-150 (SAMD21, 32KB SRAM, 256KB Flash)
 *    Dynamixel XM430-W350-R (ID 1, Protocol 2.0)
 *      open  = 60.0 deg
 *      close = 110.0 deg  (intentionally limited so fingertips don't collide)
 *
 *  Motor-field profile:
 *    The Dynamixel rotor magnet creates a position-dependent field at the
 *    sensors that swamps real contact signal. To remove it, send 'r' over
 *    serial AT THE START OF EVERY SESSION (with no object in the hand).
 *    The firmware sweeps the motor open->close at 3 RPM and records raw
 *    sensor values per encoder degree. From then on, readAllSensors uses
 *    delta = raw - profile[encoder_bin] instead of delta = raw - baseline.
 *    Without calibration ('r' not sent), the firmware falls back to the
 *    static baseline captured at startup (less accurate; thresholds will
 *    be too low and false-trigger).
 *    TCA9548A I2C multiplexer at 0x70 - 2 isolated channels.
 *      Channel 0 - Index finger
 *        Sensor 0  Index  ID1  begin(0,0)  -> A1=0,A0=0  (0x14)
 *        Sensor 1  Index  ID2  begin(0,1)  -> A1=0,A0=1  (0x15)
 *      Channel 1 - Thumb finger
 *        Sensor 2  Thumb  ID3  begin(1,0)  -> A1=1,A0=0  (0x16)
 *        Sensor 3  Thumb  ID4  begin(1,1)  -> A1=1,A0=1  (0x17)
 *
 *  Sensor amplification (set in initSensors after begin()):
 *      setGainSel(2)            -> 3x gain XY / 4.8x gain Z
 *      setResolution(0,0,0)     -> finest LSB on all axes
 *      setOverSampling(2)       -> 4x ADC averaging
 *      setDigitalFiltering(2)   -> on-chip IIR
 *      setTemperatureCompensation(1) -> reduce thermal drift
 *  These settings are now safe to use because the middle-finger sensors
 *  (the ones that saturated near the motor magnet) are no longer wired in.
 *
 *  Required libraries:
 *    arduino-MLX90393 by Theodore Yapo
 *    Dynamixel2Arduino by ROBOTIS
 *
 *  Workflow:
 *    1. Flash this sketch to the OpenRB-150.
 *    2. Run logger.py on your laptop (writes CSV to training/grasp_data).
 *    3. Type labels into logger.py, not Arduino Serial Monitor.
 *
 *  Valid labels:
 *    cube_soft, cube_stiff, cylinder_soft, cylinder_stiff, no_object
 *
 *  CSV columns (27 total):
 *    grasp_id,label,timestamp_ms,enc_deg,
 *    s0_dx,s0_dy,s0_dz, ... , s3_dx,s3_dy,s3_dz,
 *    t1_flag,t2_flag,t1_ms,t2_ms,
 *    load_t1,load_t2,current_t1,current_t2,enc_t1,time_to_stall_ms,
 *    t1_source  (0=none, 1=magnetic, 2=encoder_stall, 3=both)
 *
 *  Event definitions:
 *    t1 = first meaningful contact. Magnetic axis crosses CONTACT_THRESHOLDS,
 *         OR motor encoder stalls (soft objects that don't deflect sensors).
 *    t2 = final stable grasp / steady hold point.
 * ================================================================
 */

#include <Wire.h>
#include "MLX90393.h"
#include <Dynamixel2Arduino.h>
#include <math.h>

// ==================================================================
// TUNABLE PARAMETERS - adjust these for your setup
// ==================================================================

// Motor positions (degrees)
const float OPEN_POS_DEG       = 60.0f;
const float CLOSE_POS_DEG      = 110.0f;
const float GRASP_SPEED_RPM    = 10.0f;
// Slower speed used only during motor-field profile calibration sweep.
// At 3 RPM = 18 deg/s, a 60->120 sweep takes ~3.3 s and leaves a few samples per
// 1 deg bin so the profile is well-populated.
const float CALIB_SPEED_RPM    = 3.0f;

// Grasp timing (ms)
const uint32_t PRE_GRASP_DELAY_MS = 2000;  // Time to place object after label is sent
const uint32_t GRASP_TIMEOUT_MS   = 5000;  // Max time waiting for steady grasp
const uint32_t HOLD_DURATION_MS   = 800;   // Hold after steady detection before final t2 capture

// Per-axis contact thresholds [sensor][0=dx, 1=dy, 2=dz].
//
// IMPORTANT: These are POST-SUBTRACTION thresholds, sized for the residual noise
// after motor_field_profile (per-encoder-bin baseline) is subtracted from the raw
// reading. They are intentionally lower than the pre-subtraction thresholds because
// once the rotor field is removed, the residual is dominated by sensor + thermal
// noise (~15-30 uT 99th percentile, per motor_field_check.py analysis).
//
// If the motor-field profile has NOT been calibrated yet (profile_calibrated == false),
// readAllSensors falls back to static baseline subtraction and these thresholds will
// be too low (false positives). Always send 'r' at the start of every session.
// Tuned 2026-05-10 round 2: empirical data showed empty no_object grasps
// fired magnetic 3/10 times under prior thresholds. Index dz axes hit
// 60+ uT during the motor sweep (motor-field residual is largest on Z),
// and s3_dz had occasional 49 uT outliers. Thresholds raised on Z axes
// in particular, and CONTACT_CONFIRM_SAMPLES bumped to 8 (160 ms sustained)
// so bursty 5-7 sample noise runs no longer satisfy the trigger condition.
// Real grasp loading lasts 200-500 ms so contact still triggers easily.
const float CONTACT_THRESHOLDS[4][3] = {
  {40.0f, 40.0f, 65.0f},  // s0 Index ID1 - dz was 40, raised above 63 uT empty peak
  {40.0f, 40.0f, 60.0f},  // s1 Index ID2 - dz raised above 58 uT empty peak
  {25.0f, 25.0f, 35.0f},  // s2 Thumb ID3 (base) - moderate raise
  {25.0f, 25.0f, 30.0f}   // s3 Thumb ID4 (TIP) - still most sensitive overall
                          //   but no longer aggressive enough to trip on
                          //   empty-grasp s3_dz outliers. Real soft contacts
                          //   typically peak 30+ uT here.
};

// Axis enable mask for contact detection only.
// false = too noisy to use for t1 triggering.
// All 12 channels are still logged and used by the classifier.
//
// 2026-05-09 update: re-enabled s0_dx, s2_dy, s2_dz (clean noise floors with
// amplification on). Kept s1_dx and s1_dy masked - their noise floors are 30-50%
// higher than peer axes (37-38 p99 vs 24-29) so they're not pulling weight.
//
// 2026-05-09 followup: after 100-grasp collection, s1_dx noise p99 dropped to
// 24 and s1_dy to 43, well within the 115 threshold (4.8x / 2.7x margin).
// Re-enabled both. Now ALL 12 axes are eligible for t1 triggering.
const bool CONTACT_ENABLED[4][3] = {
  {true,  true,  true },  // s0 Index ID1: all three
  {true,  true,  true },  // s1 Index ID2: all three
  {true,  true,  true },  // s2 Thumb ID3: all three
  {true,  true,  true }   // s3 Thumb ID4: all three
};

// Ignore first N ms (power-on spikes). Require N consecutive samples above
// threshold before confirming t1.
const uint32_t CONTACT_IGNORE_MS       = 150;
// 2026-05-10 round 2: raised 5 -> 8. Empty no_object grasps were producing
// 5-7 sample bursts of motor-residual noise that satisfied the 5-sample
// requirement (3/10 false-trigger rate). 8 samples = 160 ms sustained.
// Real contact loading phases are 200-500 ms so easily clear this.
const int      CONTACT_CONFIRM_SAMPLES = 6;

// Steady-state detection before final t2 capture
const float   ENCODER_STILL_DEG   = 0.5f;
const uint8_t ENCODER_STILL_COUNT = 5;

// Encoder stall detection for soft objects (fallback t1 trigger).
const float   ENCODER_MOVING_MIN_DEG = 1.0f;
const float   ENCODER_STALL_DEG      = 0.3f;
const uint8_t ENCODER_STALL_CONFIRM  = 4;

// Sampling rate
const uint32_t SAMPLE_INTERVAL_MS = 20;    // 50 Hz

// Baseline calibration. Longer average = cleaner baseline before each run.
const int BASELINE_SAMPLES = 80;

// Motor-field profile bins: per-encoder-degree lookup of "no-object" sensor
// reading. During a real grasp, readAllSensors subtracts profile[bin] from raw
// reading instead of the static baseline, removing the Dynamixel rotor field.
// Bin 0 = OPEN_POS_DEG, last bin = CLOSE_POS_DEG.
const int PROFILE_BIN_DEG_MIN = (int)OPEN_POS_DEG;   // 60
const int PROFILE_BIN_DEG_MAX = (int)CLOSE_POS_DEG;  // 110
const int PROFILE_NUM_BINS    = PROFILE_BIN_DEG_MAX - PROFILE_BIN_DEG_MIN + 1;  // 51

// ==================================================================
// SYSTEM CONSTANTS - do not change unless your wiring changes
// ==================================================================

#define DXL_SERIAL    Serial1
#define DEBUG_SERIAL  Serial
#define DXL_DIR_PIN   2
#define TCA_ADDR      0x70
#define MOTOR_ID      1
#define NUM_SENSORS   4

// TCA9548A channel assignments
#define TCA_CH_INDEX  0   // Index finger (sensors 0, 1)
#define TCA_CH_THUMB  1   // Thumb finger (sensors 2, 3)

// ==================================================================
// SENSOR OBJECTS
// ==================================================================

MLX90393 mlx_idx1;  // Sensor 0: Index ID1  begin(0,0)
MLX90393 mlx_idx2;  // Sensor 1: Index ID2  begin(0,1)
MLX90393 mlx_thb1;  // Sensor 2: Thumb ID3  begin(1,0)
MLX90393 mlx_thb2;  // Sensor 3: Thumb ID4  begin(1,1)

MLX90393::txyz data;
float baseline[NUM_SENSORS][3];

// ==================================================================
// DYNAMIXEL
// ==================================================================

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ==================================================================
// SENSOR PRESENCE (set in initSensors via I2C ACK probe)
// ==================================================================

bool sensor_present[NUM_SENSORS] = {false, false, false, false};

// ==================================================================
// MOTOR-FIELD PROFILE (set by 'r' / calibrateMotorFieldProfile)
// ==================================================================
// Per-encoder-degree mean of raw sensor reading with NO object.
// At runtime, deltas are computed as raw - profile[encoder_bin] instead of
// raw - static_baseline. This subtracts the position-dependent rotor field.
//
// If profile_calibrated == false, readAllSensors falls back to static baseline.
float    motor_field_profile[PROFILE_NUM_BINS][NUM_SENSORS][3];
uint16_t profile_count[PROFILE_NUM_BINS];
bool     profile_calibrated = false;

// ==================================================================
// CONTACT DETECTION STATE - reset each grasp
// ==================================================================

int  contactCounter[NUM_SENSORS][3];
bool axisContact[NUM_SENSORS][3];
bool sensorContact[NUM_SENSORS];

// ==================================================================
// ENCODER STALL DETECTION STATE - reset each grasp
// ==================================================================

uint8_t encoderStallCounter = 0;

// ==================================================================
// STATE
// ==================================================================

String object_label = "";
uint16_t grasp_id = 0;

// ==================================================================
// SETUP
// ==================================================================

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 4000) {}

  DEBUG_SERIAL.println(F("# ====================================================="));
  DEBUG_SERIAL.println(F("# Prosthetic Hand - Data Collection Firmware (4-sensor)"));
  DEBUG_SERIAL.println(F("# ====================================================="));

  Wire.begin();

  // Give MLX90393 chips time to wake up after VDD rises. Without this, the
  // first I2C probe can NACK on perfectly healthy hardware - that's the
  // "sometimes detected after power cycle" symptom.
  delay(150);

  // Reset the TCA9548A mux to a known state (all channels disabled). A previous
  // run may have left a random channel selected, which corrupts the first scan.
  tcaDisableAll();

  // Diagnostic I2C scan. Expected:
  //   ch0 -> 0x14 0x15 (Index ID1, ID2)
  //   ch1 -> 0x16 0x17 (Thumb ID3, ID4)
  DEBUG_SERIAL.println(F("# Pre-init I2C scan:"));
  i2cScan(0);
  i2cScan(1);

  initMotor();
  initSensors();
  clearMotorFieldProfile();   // start uncalibrated; user must send 'r' for full sweep
  calibrateBaseline();        // static baseline (fallback if user skips 'r')
  moveToOpen();
  delay(700);

  DEBUG_SERIAL.println(F("# READY."));
  DEBUG_SERIAL.println(F("# Use logger.py to send labels. Do not open Arduino Serial Monitor at the same time."));
  DEBUG_SERIAL.println(F("# Valid labels: cube_soft | cube_stiff | cylinder_soft | cylinder_stiff | no_object"));
  DEBUG_SERIAL.println(F("# Commands: r = motor-field profile sweep (REQUIRED), h = print CSV header"));
  DEBUG_SERIAL.println(F("# WARNING: motor-field profile NOT calibrated yet. Send 'r' before any grasps"));
  DEBUG_SERIAL.println(F("# for accurate magnetic detection. Without it, contact thresholds will be too low."));
  printCSVHeader();
}

// ==================================================================
// MAIN LOOP
// ==================================================================

void loop() {
  if (!DEBUG_SERIAL.available()) {
    return;
  }

  String input = DEBUG_SERIAL.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) {
    return;
  }

  if (input == "r" || input == "R") {
    // 'r' now triggers motor-field profile calibration (sweep open->close
    // sampling raw sensor values per encoder bin). The static baseline at
    // open was already captured at startup as a fallback.
    calibrateMotorFieldProfile();
    DEBUG_SERIAL.println(F("# READY."));
    return;
  }

  if (input == "h" || input == "H") {
    printCSVHeader();
    DEBUG_SERIAL.println(F("# READY."));
    return;
  }

  if (!isValidLabel(input)) {
    DEBUG_SERIAL.print(F("# ERROR: Invalid label rejected: "));
    DEBUG_SERIAL.println(input);
    DEBUG_SERIAL.println(F("# Valid labels: cube_soft | cube_stiff | cylinder_soft | cylinder_stiff | no_object"));
    DEBUG_SERIAL.println(F("# READY."));
    return;
  }

  object_label = input;
  grasp_id++;

  DEBUG_SERIAL.print(F("# Label accepted: "));
  DEBUG_SERIAL.println(object_label);
  DEBUG_SERIAL.print(F("# Grasp ID: "));
  DEBUG_SERIAL.println(grasp_id);
  DEBUG_SERIAL.print(F("# Starting in "));
  DEBUG_SERIAL.print(PRE_GRASP_DELAY_MS / 1000);
  DEBUG_SERIAL.println(F(" s. Place the object now."));

  delay(PRE_GRASP_DELAY_MS);
  runGrasp();

  DEBUG_SERIAL.println(F("# READY. Type the next label in logger.py."));
  DEBUG_SERIAL.println(F("# ---"));
}

bool isValidLabel(const String &label) {
  return label == "cube_soft"      ||
         label == "cube_stiff"     ||
         label == "cylinder_soft"  ||
         label == "cylinder_stiff" ||
         label == "no_object";
}

// ==================================================================
// GRASP SEQUENCE
// ==================================================================

void runGrasp() {
  DEBUG_SERIAL.println(F("# Grasp start."));

  resetContactDetection();
  encoderStallCounter = 0;

  bool t1_detected = false;
  bool t2_captured = false;
  bool stable_detected = false;

  uint32_t t1_ms = 0;
  uint32_t t2_ms = 0;
  uint8_t still_count = 0;

  float load_t1 = 0.0f,    load_t2 = 0.0f;
  float current_t1 = 0.0f, current_t2 = 0.0f;
  float enc_t1 = 0.0f;
  uint32_t time_to_stall_ms = 0;
  uint8_t t1_source = 0;  // 0=none, 1=magnetic, 2=encoder_stall, 3=both

  float enc_start = getEncoderDeg();
  float prev_enc  = enc_start;

  uint32_t grasp_start = millis();
  uint32_t timeout_end = grasp_start + GRASP_TIMEOUT_MS;
  uint32_t next_sample = millis();
  uint32_t hold_start = 0;

  dxl.setGoalPosition(MOTOR_ID, CLOSE_POS_DEG, UNIT_DEGREE);

  while (true) {
    uint32_t now = millis();

    if (!stable_detected && now > timeout_end) {
      DEBUG_SERIAL.println(F("# WARNING: Grasp timeout. Final t2 was not captured. This grasp will be skipped during training."));
      break;
    }

    if (now < next_sample) {
      continue;
    }
    next_sample += SAMPLE_INTERVAL_MS;

    uint32_t ts = now - grasp_start;
    float enc = getEncoderDeg();

    float dx[NUM_SENSORS], dy[NUM_SENSORS], dz[NUM_SENSORS];
    readAllSensors(dx, dy, dz);

    bool t1_flag_this_sample = false;
    bool t2_flag_this_sample = false;

    // ---- t1 detection: dual path (magnetic primary + encoder stall backup) ----
    if (!t1_detected) {
      bool magnetic_hit = false;
      int contact_sensor = updateStableContactDetection(dx, dy, dz, ts);
      if (contact_sensor >= 0) magnetic_hit = true;

      bool stall_hit = false;
      bool past_ignore_window = (ts >= CONTACT_IGNORE_MS);
      bool hand_has_moved     = (fabs(enc - enc_start) >= ENCODER_MOVING_MIN_DEG);
      if (past_ignore_window && hand_has_moved) {
        float delta_enc = fabs(enc - prev_enc);
        if (delta_enc < ENCODER_STALL_DEG) {
          encoderStallCounter++;
          if (encoderStallCounter >= ENCODER_STALL_CONFIRM) stall_hit = true;
        } else {
          encoderStallCounter = 0;
        }
      } else {
        encoderStallCounter = 0;
      }

      if (magnetic_hit || stall_hit) {
        t1_detected         = true;
        t1_flag_this_sample = true;
        t1_ms               = ts;
        enc_t1              = enc;
        load_t1    = (float)dxl.readControlTableItem(PRESENT_LOAD,    MOTOR_ID);
        current_t1 = (float)dxl.readControlTableItem(PRESENT_CURRENT, MOTOR_ID);

        if (magnetic_hit && stall_hit)      t1_source = 3;
        else if (magnetic_hit)              t1_source = 1;
        else                                t1_source = 2;

        DEBUG_SERIAL.print(F("# t1 detected ("));
        if (t1_source == 1) DEBUG_SERIAL.print(F("magnetic"));
        else if (t1_source == 2) DEBUG_SERIAL.print(F("encoder_stall"));
        else DEBUG_SERIAL.print(F("magnetic+encoder_stall"));
        DEBUG_SERIAL.print(F(") at "));
        DEBUG_SERIAL.print(t1_ms);
        DEBUG_SERIAL.print(F(" ms, sensor="));
        DEBUG_SERIAL.print(contact_sensor);
        DEBUG_SERIAL.print(F(" enc="));
        DEBUG_SERIAL.print(enc_t1, 2);
        DEBUG_SERIAL.print(F(" load="));
        DEBUG_SERIAL.print(load_t1, 1);
        DEBUG_SERIAL.print(F(" current="));
        DEBUG_SERIAL.println(current_t1, 1);
      }
    }

    // Detect stable motor state
    if (t1_detected && !stable_detected) {
      float delta_enc = fabs(enc - prev_enc);
      if (delta_enc < ENCODER_STILL_DEG) {
        still_count++;
        if (still_count >= ENCODER_STILL_COUNT) {
          stable_detected    = true;
          hold_start         = now;
          time_to_stall_ms   = ts;
          DEBUG_SERIAL.print(F("# Stable grasp detected at "));
          DEBUG_SERIAL.print(ts);
          DEBUG_SERIAL.println(F(" ms. Holding before final t2 capture."));
        }
      } else {
        still_count = 0;
      }
    }

    if (stable_detected && !t2_captured && (now - hold_start >= HOLD_DURATION_MS)) {
      t2_captured         = true;
      t2_flag_this_sample = true;
      t2_ms               = ts;
      load_t2    = (float)dxl.readControlTableItem(PRESENT_LOAD,    MOTOR_ID);
      current_t2 = (float)dxl.readControlTableItem(PRESENT_CURRENT, MOTOR_ID);
      DEBUG_SERIAL.print(F("# t2 captured at "));
      DEBUG_SERIAL.print(t2_ms);
      DEBUG_SERIAL.print(F(" ms enc="));
      DEBUG_SERIAL.print(enc, 2);
      DEBUG_SERIAL.print(F(" load="));
      DEBUG_SERIAL.println(load_t2, 1);
    }

    printCSVRow(ts, enc, dx, dy, dz,
                t1_flag_this_sample, t2_flag_this_sample,
                t1_ms, t2_ms,
                load_t1, load_t2, current_t1, current_t2,
                enc_t1, time_to_stall_ms, t1_source);

    prev_enc = enc;

    if (t2_captured) {
      break;
    }
  }

  moveToOpen();
  delay(800);

  if (t1_detected && t2_captured) {
    DEBUG_SERIAL.println(F("# Grasp complete and valid."));
  } else {
    DEBUG_SERIAL.println(F("# Grasp complete but invalid or incomplete."));
  }
}

void resetContactDetection() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorContact[i] = false;
    for (int j = 0; j < 3; j++) {
      axisContact[i][j]    = false;
      contactCounter[i][j] = 0;
    }
  }
}

// Returns first newly-confirmed sensor index, or -1.
int updateStableContactDetection(float dx[], float dy[], float dz[], uint32_t elapsedMs) {
  if (elapsedMs < CONTACT_IGNORE_MS) return -1;
  float vals[3];
  int firstNew = -1;
  for (int i = 0; i < NUM_SENSORS; i++) {
    vals[0] = dx[i]; vals[1] = dy[i]; vals[2] = dz[i];
    for (int j = 0; j < 3; j++) {
      if (!CONTACT_ENABLED[i][j] || axisContact[i][j]) continue;
      if (fabs(vals[j]) >= CONTACT_THRESHOLDS[i][j]) {
        if (++contactCounter[i][j] >= CONTACT_CONFIRM_SAMPLES) {
          axisContact[i][j] = true;
          if (!sensorContact[i]) {
            sensorContact[i] = true;
            if (firstNew < 0) firstNew = i;
          }
        }
      } else {
        contactCounter[i][j] = 0;
      }
    }
  }
  return firstNew;
}

// ==================================================================
// SENSOR FUNCTIONS
// ==================================================================

void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(100);  // TCA9548A switching settle
}

// Diagnostic only: print I2C devices responding on a TCA channel.
void i2cScan(uint8_t channel) {
  tcaselect(channel);
  DEBUG_SERIAL.print(F("# I2C scan TCA ch "));
  DEBUG_SERIAL.print(channel);
  DEBUG_SERIAL.print(F(":"));
  bool any = false;
  for (uint8_t addr = 0x10; addr <= 0x1F; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      DEBUG_SERIAL.print(F(" 0x"));
      DEBUG_SERIAL.print(addr, HEX);
      any = true;
    }
  }
  if (!any) DEBUG_SERIAL.print(F(" (no devices)"));
  DEBUG_SERIAL.println();
}

// Map a continuous encoder reading to the nearest profile bin index.
// Clamps to [0, PROFILE_NUM_BINS-1] for out-of-range values.
inline int encToProfileBin(float enc) {
  int bin = (int)(enc + 0.5f) - PROFILE_BIN_DEG_MIN;  // round to nearest, then offset
  if (bin < 0) return 0;
  if (bin >= PROFILE_NUM_BINS) return PROFILE_NUM_BINS - 1;
  return bin;
}

// Zero out the motor-field profile and mark uncalibrated.
void clearMotorFieldProfile() {
  for (int b = 0; b < PROFILE_NUM_BINS; b++) {
    profile_count[b] = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      for (int j = 0; j < 3; j++) {
        motor_field_profile[b][i][j] = 0.0f;
      }
    }
  }
  profile_calibrated = false;
}

// Fill empty bins with the nearest populated bin (simple gap-fill so that
// bins not covered during the sweep still have a usable value).
void fillEmptyProfileBins() {
  for (int b = 0; b < PROFILE_NUM_BINS; b++) {
    if (profile_count[b] > 0) continue;
    int nearest = -1;
    int nearest_dist = PROFILE_NUM_BINS + 1;
    for (int b2 = 0; b2 < PROFILE_NUM_BINS; b2++) {
      if (profile_count[b2] == 0) continue;
      int d = (b2 > b) ? (b2 - b) : (b - b2);
      if (d < nearest_dist) { nearest_dist = d; nearest = b2; }
    }
    if (nearest >= 0) {
      for (int i = 0; i < NUM_SENSORS; i++)
        for (int j = 0; j < 3; j++)
          motor_field_profile[b][i][j] = motor_field_profile[nearest][i][j];
    }
  }
}

// Read raw (un-baselined) sensor data into a 4x3 array. Used during calibration.
void readAllSensorsRaw(float rawX[], float rawY[], float rawZ[]) {
  tcaselect(TCA_CH_INDEX);
  if (sensor_present[0]) { mlx_idx1.readData(data); rawX[0] = data.x; rawY[0] = data.y; rawZ[0] = data.z; }
  else { rawX[0] = 0; rawY[0] = 0; rawZ[0] = 0; }
  if (sensor_present[1]) { mlx_idx2.readData(data); rawX[1] = data.x; rawY[1] = data.y; rawZ[1] = data.z; }
  else { rawX[1] = 0; rawY[1] = 0; rawZ[1] = 0; }
  tcaselect(TCA_CH_THUMB);
  if (sensor_present[2]) { mlx_thb1.readData(data); rawX[2] = data.x; rawY[2] = data.y; rawZ[2] = data.z; }
  else { rawX[2] = 0; rawY[2] = 0; rawZ[2] = 0; }
  if (sensor_present[3]) { mlx_thb2.readData(data); rawX[3] = data.x; rawY[3] = data.y; rawZ[3] = data.z; }
  else { rawX[3] = 0; rawY[3] = 0; rawZ[3] = 0; }
}

// Sweep motor open->close at calibration speed, sampling sensor values per
// encoder-degree bin. Returns true on success. Aborts and returns false if
// user does not confirm with 'y'. The motor must have torque on (initMotor).
//
// IMPORTANT: must be called with NO OBJECT in the hand. Otherwise the profile
// captures object signal as part of the "motor field" and all later grasps
// will subtract that, making real contacts disappear.
bool calibrateMotorFieldProfile() {
  DEBUG_SERIAL.println(F("# ============================================================"));
  DEBUG_SERIAL.println(F("# MOTOR-FIELD PROFILE CALIBRATION"));
  DEBUG_SERIAL.println(F("# REMOVE ALL OBJECTS from the hand."));
  DEBUG_SERIAL.println(F("# The motor will sweep slowly from open to close."));
  DEBUG_SERIAL.println(F("# Send 'y' to begin or any other key to cancel."));
  DEBUG_SERIAL.println(F("# ============================================================"));

  // Drain any pending serial input
  while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();

  // Wait up to 30 seconds for confirmation
  uint32_t deadline = millis() + 30000UL;
  while (!DEBUG_SERIAL.available()) {
    if (millis() > deadline) {
      DEBUG_SERIAL.println(F("# Calibration timed out (no confirm). Aborted."));
      return false;
    }
    delay(20);
  }
  String resp = DEBUG_SERIAL.readStringUntil('\n');
  resp.trim();
  if (resp != "y" && resp != "Y") {
    DEBUG_SERIAL.println(F("# Calibration aborted by user."));
    return false;
  }

  clearMotorFieldProfile();

  // Move to open and let the motor settle.
  moveToOpen();
  delay(800);

  // Slow motor for the sweep.
  int32_t slow_vel = (int32_t)(CALIB_SPEED_RPM / 0.229f);
  int32_t fast_vel = (int32_t)(GRASP_SPEED_RPM / 0.229f);
  dxl.writeControlTableItem(PROFILE_VELOCITY, MOTOR_ID, slow_vel);

  // Drive toward close.
  dxl.setGoalPosition(MOTOR_ID, CLOSE_POS_DEG, UNIT_DEGREE);

  uint32_t sweep_start  = millis();
  uint32_t sweep_end    = sweep_start + 8000UL;  // generous timeout
  uint32_t next_sample  = millis();
  uint16_t total        = 0;

  float rawX[NUM_SENSORS], rawY[NUM_SENSORS], rawZ[NUM_SENSORS];

  while (millis() < sweep_end) {
    uint32_t now = millis();
    if (now < next_sample) continue;
    next_sample += SAMPLE_INTERVAL_MS;

    float enc = getEncoderDeg();
    if (enc >= CLOSE_POS_DEG - 0.3f) break;
    if (enc < OPEN_POS_DEG - 1.0f)   continue;  // ignore stray reads below open

    int bin = encToProfileBin(enc);
    readAllSensorsRaw(rawX, rawY, rawZ);

    // Online running mean update (no large sum buffers needed).
    profile_count[bin]++;
    uint16_t n = profile_count[bin];
    for (int i = 0; i < NUM_SENSORS; i++) {
      motor_field_profile[bin][i][0] += (rawX[i] - motor_field_profile[bin][i][0]) / (float)n;
      motor_field_profile[bin][i][1] += (rawY[i] - motor_field_profile[bin][i][1]) / (float)n;
      motor_field_profile[bin][i][2] += (rawZ[i] - motor_field_profile[bin][i][2]) / (float)n;
    }
    total++;
  }

  // Restore normal grasp speed and return to open.
  dxl.writeControlTableItem(PROFILE_VELOCITY, MOTOR_ID, fast_vel);
  moveToOpen();
  delay(800);

  // Gap-fill any bins we skipped (e.g. motor accelerating through them).
  fillEmptyProfileBins();

  // Count populated bins for reporting.
  int populated = 0;
  for (int b = 0; b < PROFILE_NUM_BINS; b++) if (profile_count[b] > 0) populated++;

  profile_calibrated = (populated >= PROFILE_NUM_BINS / 2);

  DEBUG_SERIAL.print(F("# Profile sweep done. samples="));
  DEBUG_SERIAL.print(total);
  DEBUG_SERIAL.print(F(" populated_bins="));
  DEBUG_SERIAL.print(populated);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.print(PROFILE_NUM_BINS);
  if (profile_calibrated) {
    DEBUG_SERIAL.println(F("  CALIBRATED."));
  } else {
    DEBUG_SERIAL.println(F("  TOO SPARSE - calibration FAILED. Try again."));
  }
  return profile_calibrated;
}

// Apply amplification + filtering settings to each MLX90393.
//   gain=2 -> 3x XY / 4.8x Z
//   res(0,0,0) -> finest LSB on all axes
//   OSR=2 -> 4x ADC averaging
//   DF=2 -> on-chip IIR
//   TempComp=1 -> drift reduction
inline void applyAmpSettings(MLX90393& s) {
  s.setGainSel(2);
  s.setResolution(0, 0, 0);
  s.setOverSampling(2);
  s.setDigitalFiltering(2);
  s.setTemperatureCompensation(1);
}

// Probe the I2C bus for one address; return true on ACK.
inline bool addrResponds(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Disable all TCA9548A channels - puts the mux into a known state. Useful at
// boot since a previous run may have left a random channel selected.
inline void tcaDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delayMicroseconds(100);
}

// Initialize one sensor with retries. The MLX90393 needs a few ms after VDD
// rises before it ACKs I2C; without retry, the first probe right after
// Wire.begin() can NACK on healthy hardware. We try up to 4 times with
// 50 ms gaps, then give up cleanly so missing chips don't hang the firmware.
inline bool initOne(MLX90393& s, uint8_t a1, uint8_t a0,
                    int idx, const char* name) {
  uint8_t addr = 0x14 + (a1 << 1) + a0;
  const int RETRIES = 4;
  for (int attempt = 1; attempt <= RETRIES; attempt++) {
    if (addrResponds(addr)) {
      uint8_t st = s.begin(a1, a0);
      applyAmpSettings(s);
      sensor_present[idx] = (st == MLX90393::STATUS_OK);
      DEBUG_SERIAL.print(F("#   "));
      DEBUG_SERIAL.print(name);
      DEBUG_SERIAL.print(F(" begin status: 0x"));
      DEBUG_SERIAL.print(st, HEX);
      if (attempt > 1) {
        DEBUG_SERIAL.print(F(" (attempt "));
        DEBUG_SERIAL.print(attempt);
        DEBUG_SERIAL.print(F(")"));
      }
      DEBUG_SERIAL.println();
      return sensor_present[idx];
    }
    delay(50);
  }
  sensor_present[idx] = false;
  DEBUG_SERIAL.print(F("#   "));
  DEBUG_SERIAL.print(name);
  DEBUG_SERIAL.print(F("  NOT FOUND at 0x"));
  DEBUG_SERIAL.print(addr, HEX);
  DEBUG_SERIAL.print(F(" after "));
  DEBUG_SERIAL.print(RETRIES);
  DEBUG_SERIAL.println(F(" attempts - check power/wiring"));
  return false;
}

void initSensors() {
  DEBUG_SERIAL.println(F("# Initialising sensors..."));

  // Channel 0 - Index finger
  tcaselect(TCA_CH_INDEX);
  initOne(mlx_idx1, 0, 0, 0, "Idx1 (ID1)");
  initOne(mlx_idx2, 0, 1, 1, "Idx2 (ID2)");

  // Channel 1 - Thumb finger
  tcaselect(TCA_CH_THUMB);
  initOne(mlx_thb1, 1, 0, 2, "Thb1 (ID3)");
  initOne(mlx_thb2, 1, 1, 3, "Thb2 (ID4)");

  int alive = 0;
  for (int i = 0; i < NUM_SENSORS; i++) if (sensor_present[i]) alive++;
  DEBUG_SERIAL.print(F("# Sensors alive: "));
  DEBUG_SERIAL.print(alive);
  DEBUG_SERIAL.print(F("/"));
  DEBUG_SERIAL.println(NUM_SENSORS);
  if (alive < NUM_SENSORS) {
    DEBUG_SERIAL.println(F("# WARNING: missing sensors will report zeros in CSV. Fix hardware to recover."));
  }

  delay(100);
  DEBUG_SERIAL.println(F("# Sensors OK (gain=2, res=0/0/0, OSR=2, DF=2, TCMP=on)."));
}

void calibrateBaseline() {
  DEBUG_SERIAL.println(F("# Calibrating baseline. Keep fingers open with no object."));
  float sumX[NUM_SENSORS] = {0};
  float sumY[NUM_SENSORS] = {0};
  float sumZ[NUM_SENSORS] = {0};

  for (int s = 0; s < BASELINE_SAMPLES; s++) {
    // Channel 0 - Index finger (sensors 0, 1)
    tcaselect(TCA_CH_INDEX);
    if (sensor_present[0]) { mlx_idx1.readData(data); sumX[0] += data.x; sumY[0] += data.y; sumZ[0] += data.z; }
    if (sensor_present[1]) { mlx_idx2.readData(data); sumX[1] += data.x; sumY[1] += data.y; sumZ[1] += data.z; }

    // Channel 1 - Thumb finger (sensors 2, 3)
    tcaselect(TCA_CH_THUMB);
    if (sensor_present[2]) { mlx_thb1.readData(data); sumX[2] += data.x; sumY[2] += data.y; sumZ[2] += data.z; }
    if (sensor_present[3]) { mlx_thb2.readData(data); sumX[3] += data.x; sumY[3] += data.y; sumZ[3] += data.z; }

    delay(SAMPLE_INTERVAL_MS);
  }

  const char* names[NUM_SENSORS] = {"Idx1", "Idx2", "Thb1", "Thb2"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensor_present[i]) {
      baseline[i][0] = 0;
      baseline[i][1] = 0;
      baseline[i][2] = 0;
      DEBUG_SERIAL.print(F("# Baseline "));
      DEBUG_SERIAL.print(names[i]);
      DEBUG_SERIAL.println(F("  (missing - forced to 0)"));
      continue;
    }
    baseline[i][0] = sumX[i] / BASELINE_SAMPLES;
    baseline[i][1] = sumY[i] / BASELINE_SAMPLES;
    baseline[i][2] = sumZ[i] / BASELINE_SAMPLES;

    DEBUG_SERIAL.print(F("# Baseline "));
    DEBUG_SERIAL.print(names[i]);
    DEBUG_SERIAL.print(F(" X=")); DEBUG_SERIAL.print(baseline[i][0], 2);
    DEBUG_SERIAL.print(F(" Y=")); DEBUG_SERIAL.print(baseline[i][1], 2);
    DEBUG_SERIAL.print(F(" Z=")); DEBUG_SERIAL.println(baseline[i][2], 2);
  }
  DEBUG_SERIAL.println(F("# Baseline done."));
}

// Computes the baseline (raw value to subtract) for one sensor at the current
// encoder position. If the motor-field profile has been calibrated, returns
// the per-bin lookup; otherwise falls back to the static open-position baseline.
inline void getBaseline(int sensor_idx, int bin, float* bx, float* by, float* bz) {
  if (profile_calibrated) {
    *bx = motor_field_profile[bin][sensor_idx][0];
    *by = motor_field_profile[bin][sensor_idx][1];
    *bz = motor_field_profile[bin][sensor_idx][2];
  } else {
    *bx = baseline[sensor_idx][0];
    *by = baseline[sensor_idx][1];
    *bz = baseline[sensor_idx][2];
  }
}

void readAllSensors(float dx[], float dy[], float dz[]) {
  float enc = getEncoderDeg();
  int bin = encToProfileBin(enc);
  float bx, by, bz;

  // Channel 0 - Index finger (sensors 0, 1)
  tcaselect(TCA_CH_INDEX);
  if (sensor_present[0]) {
    mlx_idx1.readData(data);
    getBaseline(0, bin, &bx, &by, &bz);
    dx[0] = data.x - bx; dy[0] = data.y - by; dz[0] = data.z - bz;
  } else { dx[0] = 0; dy[0] = 0; dz[0] = 0; }
  if (sensor_present[1]) {
    mlx_idx2.readData(data);
    getBaseline(1, bin, &bx, &by, &bz);
    dx[1] = data.x - bx; dy[1] = data.y - by; dz[1] = data.z - bz;
  } else { dx[1] = 0; dy[1] = 0; dz[1] = 0; }

  // Channel 1 - Thumb finger (sensors 2, 3)
  tcaselect(TCA_CH_THUMB);
  if (sensor_present[2]) {
    mlx_thb1.readData(data);
    getBaseline(2, bin, &bx, &by, &bz);
    dx[2] = data.x - bx; dy[2] = data.y - by; dz[2] = data.z - bz;
  } else { dx[2] = 0; dy[2] = 0; dz[2] = 0; }
  if (sensor_present[3]) {
    mlx_thb2.readData(data);
    getBaseline(3, bin, &bx, &by, &bz);
    dx[3] = data.x - bx; dy[3] = data.y - by; dz[3] = data.z - bz;
  } else { dx[3] = 0; dy[3] = 0; dz[3] = 0; }
}

// ==================================================================
// MOTOR FUNCTIONS
// ==================================================================

void initMotor() {
  DEBUG_SERIAL.println(F("# Initialising motor..."));
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  if (!dxl.ping(MOTOR_ID)) {
    DEBUG_SERIAL.println(F("# ERROR: XM430 not found. Check power, cable, ID=1 and baud=57600."));
    while (true) delay(1000);
  }

  dxl.torqueOff(MOTOR_ID);
  dxl.setOperatingMode(MOTOR_ID, OP_POSITION);

  int32_t vel_raw = (int32_t)(GRASP_SPEED_RPM / 0.229f);
  dxl.writeControlTableItem(PROFILE_VELOCITY, MOTOR_ID, vel_raw);

  dxl.torqueOn(MOTOR_ID);
  DEBUG_SERIAL.println(F("# Motor OK."));
}

void moveToOpen() {
  dxl.setGoalPosition(MOTOR_ID, OPEN_POS_DEG, UNIT_DEGREE);
}

float getEncoderDeg() {
  return dxl.getPresentPosition(MOTOR_ID, UNIT_DEGREE);
}

// ==================================================================
// CSV OUTPUT
// ==================================================================

void printCSVHeader() {
  DEBUG_SERIAL.println(
    "grasp_id,label,timestamp_ms,enc_deg,"
    "s0_dx,s0_dy,s0_dz,"
    "s1_dx,s1_dy,s1_dz,"
    "s2_dx,s2_dy,s2_dz,"
    "s3_dx,s3_dy,s3_dz,"
    "t1_flag,t2_flag,t1_ms,t2_ms,"
    "load_t1,load_t2,current_t1,current_t2,enc_t1,time_to_stall_ms,"
    "t1_source"
  );
}

void printCSVRow(uint32_t ts, float enc,
                 float dx[], float dy[], float dz[],
                 bool t1_flag, bool t2_flag,
                 uint32_t t1t, uint32_t t2t,
                 float load_t1, float load_t2,
                 float current_t1, float current_t2,
                 float enc_t1, uint32_t time_to_stall_ms,
                 uint8_t t1_source) {
  DEBUG_SERIAL.print(grasp_id);      DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(object_label);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(ts);            DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(enc, 3);        DEBUG_SERIAL.print(',');

  for (int i = 0; i < NUM_SENSORS; i++) {
    DEBUG_SERIAL.print(dx[i], 3); DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(dy[i], 3); DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(dz[i], 3); DEBUG_SERIAL.print(',');
  }

  DEBUG_SERIAL.print(t1_flag ? 1 : 0); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(t2_flag ? 1 : 0); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(t1t);             DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(t2t);             DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(load_t1, 1);      DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(load_t2, 1);      DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(current_t1, 1);   DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(current_t2, 1);   DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(enc_t1, 3);       DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(time_to_stall_ms); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.println(t1_source);
}

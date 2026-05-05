/*
 * ================================================================
 *  PROSTHETIC HAND - DATA COLLECTION FIRMWARE
 *  File: data_collection.ino
 * ================================================================
 *
 *  Purpose:
 *    Collect labelled single-grasp tactile data for object recognition.
 *
 *  Hardware layout:
 *    OpenRB-150 (SAMD21, 32KB SRAM, 256KB Flash)
 *    Dynamixel XM430-W350-R (ID 1, Protocol 2.0)
 *      open  = 33.0 deg
 *      close = 90.0 deg
 *    TCA9548A I2C multiplexer at 0x70
 *      Channel 0 - Strip 1 (Index + Thumb fingers)
 *        Sensor 0  Index  ID1  begin(0,0)  -> A1=0,A0=0
 *        Sensor 1  Index  ID2  begin(0,1)  -> A1=0,A0=1
 *        Sensor 2  Thumb  ID3  begin(1,0)  -> A1=1,A0=0
 *        Sensor 3  Thumb  ID4  begin(1,1)  -> A1=1,A0=1
 *      Channel 1 - Strip 2 (Middle finger)
 *        Sensor 4  Middle ID1  begin(0,0)  -> A1=0,A0=0
 *        Sensor 5  Middle ID2  begin(0,1)  -> A1=0,A0=1
 *
 *  Required libraries:
 *    arduino-MLX90393 by Theodore Yapo
 *    Dynamixel2Arduino by ROBOTIS
 *
 *  Correct workflow:
 *    1. Flash this sketch to the OpenRB-150.
 *    2. Run logger.py on your laptop.
 *    3. Type labels into logger.py, not Arduino Serial Monitor.
 *
 *  Valid labels:
 *    cylinder_stiff, cylinder_soft, cube_stiff, cube_soft
 *
 *  CSV columns:
 *    grasp_id,label,timestamp_ms,enc_deg,
 *    s0_dx,s0_dy,s0_dz, ... , s5_dx,s5_dy,s5_dz,
 *    t1_flag,t2_flag,t1_ms,t2_ms
 *
 *  Event definitions:
 *    t1 = first meaningful contact. It is detected when any magnetic
 *         sensor axis deviates from baseline by CONTACT_THRESHOLD.
 *    t2 = final stable grasp / steady hold point. The motor must first
 *         become steady, then the hand holds briefly. The final sample
 *         at the end of that hold is flagged as t2. This matches the
 *         training script, which uses the final t2 row for each grasp.
 *
 *  Lines beginning with '#' are status/debug messages. logger.py prints
 *  them to the terminal but does not write them to the CSV file.
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
const float CLOSE_POS_DEG      = 115.0f;
const float GRASP_SPEED_RPM    = 10.0f;

// Grasp timing (ms)
const uint32_t PRE_GRASP_DELAY_MS = 2000;  // Time to place object after label is sent
const uint32_t GRASP_TIMEOUT_MS   = 5000;  // Max time waiting for steady grasp
const uint32_t HOLD_DURATION_MS   = 800;   // Hold after steady detection before final t2 capture

// Contact detection for t1
// Start with empty grasps, find the typical max noise in the CSV,
// then set this to roughly 3-5 times that noise floor.
const float CONTACT_THRESHOLD = 0.0f;

// Steady-state detection before final t2 capture
const float   ENCODER_STILL_DEG   = 0.5f;
const uint8_t ENCODER_STILL_COUNT = 5;

// Sampling rate
const uint32_t SAMPLE_INTERVAL_MS = 20;    // 50 Hz

// Baseline calibration
const int BASELINE_SAMPLES = 40;

// ==================================================================
// SYSTEM CONSTANTS - do not change unless your wiring changes
// ==================================================================

#define DXL_SERIAL    Serial1
#define DEBUG_SERIAL  Serial
#define DXL_DIR_PIN   2
#define TCA_ADDR      0x70
#define MOTOR_ID      1
#define NUM_SENSORS   6

// ==================================================================
// SENSOR OBJECTS
// ==================================================================

// Strip 1, TCA channel 0
MLX90393 mlx_idx1;  // Sensor 0: Index  ID1  begin(0,0)
MLX90393 mlx_idx2;  // Sensor 1: Index  ID2  begin(0,1)
MLX90393 mlx_thb1;  // Sensor 2: Thumb  ID3  begin(1,0)
MLX90393 mlx_thb2;  // Sensor 3: Thumb  ID4  begin(1,1)

// Strip 2, TCA channel 1
MLX90393 mlx_mid1;  // Sensor 4: Middle ID1  begin(0,0)
MLX90393 mlx_mid2;  // Sensor 5: Middle ID2  begin(0,1)

MLX90393::txyz data;
float baseline[NUM_SENSORS][3];

// ==================================================================
// DYNAMIXEL
// ==================================================================

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

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
  DEBUG_SERIAL.println(F("# Prosthetic Hand - Data Collection Firmware"));
  DEBUG_SERIAL.println(F("# ====================================================="));

  Wire.begin();
  initMotor();
  initSensors();
  calibrateBaseline();
  moveToOpen();
  delay(700);

  DEBUG_SERIAL.println(F("# READY."));
  DEBUG_SERIAL.println(F("# Use logger.py to send labels. Do not open Arduino Serial Monitor at the same time."));
  DEBUG_SERIAL.println(F("# Valid labels: cylinder_stiff | cylinder_soft | cube_stiff | cube_soft"));
  DEBUG_SERIAL.println(F("# Commands: r = recalibrate baseline, h = print CSV header"));
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
    DEBUG_SERIAL.println(F("# Recalibrating baseline. Keep hand open and remove object."));
    moveToOpen();
    delay(700);
    calibrateBaseline();
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
    DEBUG_SERIAL.println(F("# Valid labels: cylinder_stiff | cylinder_soft | cube_stiff | cube_soft"));
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
  return label == "cylinder_stiff" ||
         label == "cylinder_soft"  ||
         label == "cube_stiff"     ||
         label == "cube_soft";
}

// ==================================================================
// GRASP SEQUENCE
// ==================================================================

void runGrasp() {
  DEBUG_SERIAL.println(F("# Grasp start."));

  bool t1_detected = false;
  bool t2_captured = false;
  bool stable_detected = false;

  uint32_t t1_ms = 0;
  uint32_t t2_ms = 0;
  uint8_t still_count = 0;

  float prev_enc = getEncoderDeg();

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

    // t1 = first meaningful contact.
    if (!t1_detected) {
      int contact_sensor = detectContactSensor(dx, dy, dz);
      if (contact_sensor >= 0) {
        t1_detected = true;
        t1_flag_this_sample = true;
        t1_ms = ts;
        DEBUG_SERIAL.print(F("# t1 detected at "));
        DEBUG_SERIAL.print(t1_ms);
        DEBUG_SERIAL.print(F(" ms, sensor="));
        DEBUG_SERIAL.println(contact_sensor);
      }
    }

    // Detect stable motor state first, then keep holding. The final
    // sample after HOLD_DURATION_MS becomes t2.
    if (t1_detected && !stable_detected) {
      float delta_enc = fabs(enc - prev_enc);
      if (delta_enc < ENCODER_STILL_DEG) {
        still_count++;
        if (still_count >= ENCODER_STILL_COUNT) {
          stable_detected = true;
          hold_start = now;
          DEBUG_SERIAL.print(F("# Stable grasp detected at "));
          DEBUG_SERIAL.print(ts);
          DEBUG_SERIAL.println(F(" ms. Holding before final t2 capture."));
        }
      } else {
        still_count = 0;
      }
    }

    if (stable_detected && !t2_captured && (now - hold_start >= HOLD_DURATION_MS)) {
      t2_captured = true;
      t2_flag_this_sample = true;
      t2_ms = ts;
      DEBUG_SERIAL.print(F("# t2 captured at final stable hold point, "));
      DEBUG_SERIAL.print(t2_ms);
      DEBUG_SERIAL.println(F(" ms."));
    }

    printCSVRow(ts, enc, dx, dy, dz, t1_flag_this_sample, t2_flag_this_sample, t1_ms, t2_ms);

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

int detectContactSensor(float dx[], float dy[], float dz[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (fabs(dx[i]) > CONTACT_THRESHOLD ||
        fabs(dy[i]) > CONTACT_THRESHOLD ||
        fabs(dz[i]) > CONTACT_THRESHOLD) {
      return i;
    }
  }
  return -1;
}

// ==================================================================
// SENSOR FUNCTIONS
// ==================================================================

void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void initSensors() {
  DEBUG_SERIAL.println(F("# Initialising sensors..."));

  tcaselect(0);
  mlx_idx1.begin(0, 0); mlx_idx1.setOverSampling(0); mlx_idx1.setDigitalFiltering(0);
  mlx_idx2.begin(0, 1); mlx_idx2.setOverSampling(0); mlx_idx2.setDigitalFiltering(0);
  mlx_thb1.begin(1, 0); mlx_thb1.setOverSampling(0); mlx_thb1.setDigitalFiltering(0);
  mlx_thb2.begin(1, 1); mlx_thb2.setOverSampling(0); mlx_thb2.setDigitalFiltering(0);

  tcaselect(1);
  mlx_mid1.begin(0, 0); mlx_mid1.setOverSampling(0); mlx_mid1.setDigitalFiltering(0);
  mlx_mid2.begin(0, 1); mlx_mid2.setOverSampling(0); mlx_mid2.setDigitalFiltering(0);

  delay(100);
  DEBUG_SERIAL.println(F("# Sensors OK."));
}

void calibrateBaseline() {
  DEBUG_SERIAL.println(F("# Calibrating baseline. Keep fingers open with no object."));
  float sumX[NUM_SENSORS] = {0};
  float sumY[NUM_SENSORS] = {0};
  float sumZ[NUM_SENSORS] = {0};

  for (int s = 0; s < BASELINE_SAMPLES; s++) {
    tcaselect(0);
    mlx_idx1.readData(data); sumX[0] += data.x; sumY[0] += data.y; sumZ[0] += data.z;
    mlx_idx2.readData(data); sumX[1] += data.x; sumY[1] += data.y; sumZ[1] += data.z;
    mlx_thb1.readData(data); sumX[2] += data.x; sumY[2] += data.y; sumZ[2] += data.z;
    mlx_thb2.readData(data); sumX[3] += data.x; sumY[3] += data.y; sumZ[3] += data.z;

    tcaselect(1);
    mlx_mid1.readData(data); sumX[4] += data.x; sumY[4] += data.y; sumZ[4] += data.z;
    mlx_mid2.readData(data); sumX[5] += data.x; sumY[5] += data.y; sumZ[5] += data.z;

    delay(SAMPLE_INTERVAL_MS);
  }

  const char* names[NUM_SENSORS] = {"Idx1", "Idx2", "Thb1", "Thb2", "Mid1", "Mid2"};
  for (int i = 0; i < NUM_SENSORS; i++) {
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

void readAllSensors(float dx[], float dy[], float dz[]) {
  tcaselect(0);
  mlx_idx1.readData(data); dx[0] = data.x - baseline[0][0]; dy[0] = data.y - baseline[0][1]; dz[0] = data.z - baseline[0][2];
  mlx_idx2.readData(data); dx[1] = data.x - baseline[1][0]; dy[1] = data.y - baseline[1][1]; dz[1] = data.z - baseline[1][2];
  mlx_thb1.readData(data); dx[2] = data.x - baseline[2][0]; dy[2] = data.y - baseline[2][1]; dz[2] = data.z - baseline[2][2];
  mlx_thb2.readData(data); dx[3] = data.x - baseline[3][0]; dy[3] = data.y - baseline[3][1]; dz[3] = data.z - baseline[3][2];

  tcaselect(1);
  mlx_mid1.readData(data); dx[4] = data.x - baseline[4][0]; dy[4] = data.y - baseline[4][1]; dz[4] = data.z - baseline[4][2];
  mlx_mid2.readData(data); dx[5] = data.x - baseline[5][0]; dy[5] = data.y - baseline[5][1]; dz[5] = data.z - baseline[5][2];
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

  int32_t vel_raw = (int32_t)(GRASP_SPEED_RPM / 0.229f);  // XM430 Profile Velocity unit = 0.229 rpm
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
    "s4_dx,s4_dy,s4_dz,"
    "s5_dx,s5_dy,s5_dz,"
    "t1_flag,t2_flag,t1_ms,t2_ms"
  );
}

void printCSVRow(uint32_t ts, float enc,
                 float dx[], float dy[], float dz[],
                 bool t1_flag, bool t2_flag,
                 uint32_t t1t, uint32_t t2t) {
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
  DEBUG_SERIAL.println(t2t);
}

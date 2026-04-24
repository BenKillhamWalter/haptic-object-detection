/*
 * ================================================================
 *  PROSTHETIC HAND — DATA COLLECTION FIRMWARE
 *  File: data_collection.ino
 * ================================================================
 *
 *  Hardware layout:
 *    OpenRB-150 (SAMD21, 32KB SRAM, 256KB Flash)
 *    Dynamixel XM430-W350-R  (ID 1, Protocol 2.0)
 *      open  = 33.0 deg
 *      close = 90.0 deg
 *    TCA9548A I2C multiplexer at 0x70
 *      Channel 0 — Strip 1 (Index + Thumb fingers)
 *        Sensor 0  Index  ID1  begin(0,0)  → A1=0,A0=0
 *        Sensor 1  Index  ID2  begin(0,1)  → A1=0,A0=1
 *        Sensor 2  Thumb  ID3  begin(1,0)  → A1=1,A0=0
 *        Sensor 3  Thumb  ID4  begin(1,1)  → A1=1,A0=1
 *      Channel 1 — Strip 2 (Middle finger)
 *        Sensor 4  Middle ID1  begin(0,0)  → A1=0,A0=0
 *        Sensor 5  Middle ID2  begin(0,1)  → A1=0,A0=1
 *
 *  Libraries required:
 *    arduino-MLX90393  (Theodore Yapo — provided as zip)
 *    Dynamixel2Arduino (ROBOTIS — Arduino Library Manager)
 *
 *  Usage:
 *    1. Open Serial Monitor at 115200 baud, set line ending to Newline
 *    2. Place object, type label, press Enter:
 *         cylinder_stiff  cylinder_soft  cube_stiff  cube_soft
 *    3. Hand grasps automatically after PRE_GRASP_DELAY_MS
 *    4. Full time-series CSV streams to Serial
 *    5. Capture with logger.py running on your laptop
 *
 *  CSV columns:
 *    grasp_id, label, timestamp_ms, enc_deg,
 *    s0_dx,s0_dy,s0_dz,  s1_dx,s1_dy,s1_dz,
 *    s2_dx,s2_dy,s2_dz,  s3_dx,s3_dy,s3_dz,
 *    s4_dx,s4_dy,s4_dz,  s5_dx,s5_dy,s5_dz,
 *    t1_flag, t2_flag, t1_ms, t2_ms
 *
 *  All sensor values are deltas from baseline (contact = deviation from zero).
 *  Lines beginning with '#' are status/debug; logger.py filters them out.
 * ================================================================
 */

#include <Wire.h>
#include "MLX90393.h"
#include <Dynamixel2Arduino.h>

// ═══════════════════════════════════════════════════════════════════
//  ▶▶  TUNABLE PARAMETERS — adjust these for your setup  ◀◀
// ═══════════════════════════════════════════════════════════════════

// ── Motor positions (degrees) ─────────────────────────────────────
const float OPEN_POS_DEG       = 33.0;   // Fully open
const float CLOSE_POS_DEG      = 90.0;   // Fully closed
const float GRASP_SPEED_RPM    = 10.0;   // Slow = cleaner contact detection
                                          // Increase if grasps feel too slow

// ── Grasp timing (ms) ─────────────────────────────────────────────
const uint32_t PRE_GRASP_DELAY_MS = 2000; // Countdown after label entry (time to step back)
const uint32_t GRASP_TIMEOUT_MS   = 5000; // Max time waiting for motor to close
const uint32_t HOLD_DURATION_MS   = 800;  // How long to hold after t2 detected

// ── Contact detection (t1) ────────────────────────────────────────
// t1 fires when any sensor axis delta exceeds this value.
// To find a good value: do empty grasps, check the max noise in the CSV,
// then set threshold to 3-5x that noise floor.
const float CONTACT_THRESHOLD = 40.0;    // sensor units (µT approx)

// ── Steady-state detection (t2) ───────────────────────────────────
// t2 fires when the encoder has stopped moving.
const float   ENCODER_STILL_DEG   = 0.5; // Max movement to count as "still" (degrees)
const uint8_t ENCODER_STILL_COUNT = 5;   // Consecutive still samples required

// ── Sampling rate ─────────────────────────────────────────────────
const uint32_t SAMPLE_INTERVAL_MS = 20;  // 50 Hz — safe for 6 sensors over I2C

// ── Baseline calibration ──────────────────────────────────────────
const int BASELINE_SAMPLES = 40;         // Averaged readings for baseline (at startup)

// ═══════════════════════════════════════════════════════════════════
//  SYSTEM CONSTANTS — do not change
// ═══════════════════════════════════════════════════════════════════

#define DXL_SERIAL    Serial1
#define DEBUG_SERIAL  Serial
#define DXL_DIR_PIN   2
#define TCA_ADDR      0x70
#define MOTOR_ID      1
#define NUM_SENSORS   6

// ═══════════════════════════════════════════════════════════════════
//  SENSOR OBJECTS
// ═══════════════════════════════════════════════════════════════════

// Strip 1, TCA channel 0
MLX90393 mlx_idx1;  // Sensor 0: Index  ID1  begin(0,0)
MLX90393 mlx_idx2;  // Sensor 1: Index  ID2  begin(0,1)
MLX90393 mlx_thb1;  // Sensor 2: Thumb  ID3  begin(1,0)
MLX90393 mlx_thb2;  // Sensor 3: Thumb  ID4  begin(1,1)

// Strip 2, TCA channel 1
MLX90393 mlx_mid1;  // Sensor 4: Middle ID1  begin(0,0)
MLX90393 mlx_mid2;  // Sensor 5: Middle ID2  begin(0,1)

MLX90393::txyz data;               // Shared read buffer
float baseline[NUM_SENSORS][3];   // [sensor][0=X,1=Y,2=Z]

// ═══════════════════════════════════════════════════════════════════
//  DYNAMIXEL
// ═══════════════════════════════════════════════════════════════════

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ═══════════════════════════════════════════════════════════════════
//  STATE
// ═══════════════════════════════════════════════════════════════════

String   object_label = "";
uint16_t grasp_id     = 0;

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 4000);

  DEBUG_SERIAL.println(F("# ====================================================="));
  DEBUG_SERIAL.println(F("# Prosthetic Hand — Data Collection Firmware"));
  DEBUG_SERIAL.println(F("# ====================================================="));

  Wire.begin();
  initMotor();
  initSensors();
  calibrateBaseline();
  moveToOpen();

  DEBUG_SERIAL.println(F("# ====================================================="));
  DEBUG_SERIAL.println(F("# READY."));
  DEBUG_SERIAL.println(F("# Type label + Enter: cylinder_stiff | cylinder_soft | cube_stiff | cube_soft"));
  DEBUG_SERIAL.println(F("# ====================================================="));
  printCSVHeader();
}

// ═══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
  if (DEBUG_SERIAL.available()) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    object_label = input;
    grasp_id++;

    DEBUG_SERIAL.print(F("# Label: "));    DEBUG_SERIAL.println(object_label);
    DEBUG_SERIAL.print(F("# Grasp ID: ")); DEBUG_SERIAL.println(grasp_id);
    DEBUG_SERIAL.print(F("# Starting in "));
    DEBUG_SERIAL.print(PRE_GRASP_DELAY_MS / 1000);
    DEBUG_SERIAL.println(F("s — place object now."));

    delay(PRE_GRASP_DELAY_MS);
    runGrasp();

    DEBUG_SERIAL.println(F("# Complete. Type next label when ready."));
    DEBUG_SERIAL.println(F("# ---"));
  }
}

// ═══════════════════════════════════════════════════════════════════
//  GRASP SEQUENCE
// ═══════════════════════════════════════════════════════════════════

void runGrasp() {
  bool     t1_detected = false;
  bool     t2_detected = false;
  uint32_t t1_ms       = 0;
  uint32_t t2_ms       = 0;
  uint8_t  still_count = 0;

  float prev_enc = getEncoderDeg();

  // Command motor to close — open-loop, no force feedback
  uint32_t grasp_start = millis();
  dxl.setGoalPosition(MOTOR_ID, CLOSE_POS_DEG, UNIT_DEGREE);

  uint32_t next_sample = millis();
  uint32_t timeout_end = grasp_start + GRASP_TIMEOUT_MS;
  bool     holding     = false;
  uint32_t hold_start  = 0;

  while (true) {
    uint32_t now = millis();

    // Exit conditions
    if (holding && (now - hold_start >= HOLD_DURATION_MS)) break;
    if (!holding && (now > timeout_end)) {
      DEBUG_SERIAL.println(F("# WARNING: Grasp timeout — t2 not detected. Releasing."));
      break;
    }

    if (now < next_sample) continue;
    next_sample += SAMPLE_INTERVAL_MS;

    uint32_t ts  = now - grasp_start;
    float    enc = getEncoderDeg();

    // Read all 6 sensors (delta from baseline)
    float dx[NUM_SENSORS], dy[NUM_SENSORS], dz[NUM_SENSORS];
    readAllSensors(dx, dy, dz);

    // ── t1: First contact ─────────────────────────────────────────
    if (!t1_detected) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (fabs(dx[i]) > CONTACT_THRESHOLD ||
            fabs(dy[i]) > CONTACT_THRESHOLD ||
            fabs(dz[i]) > CONTACT_THRESHOLD) {
          t1_detected = true;
          t1_ms = ts;
          DEBUG_SERIAL.print(F("# t1 at "));
          DEBUG_SERIAL.print(ts);
          DEBUG_SERIAL.print(F(" ms  sensor="));
          DEBUG_SERIAL.println(i);
          break;
        }
      }
    }

    // ── t2: Encoder stopped moving ────────────────────────────────
    if (t1_detected && !t2_detected) {
      float delta = fabs(enc - prev_enc);
      if (delta < ENCODER_STILL_DEG) {
        still_count++;
        if (still_count >= ENCODER_STILL_COUNT) {
          t2_detected = true;
          t2_ms = ts;
          holding    = true;
          hold_start = now;
          DEBUG_SERIAL.print(F("# t2 at "));
          DEBUG_SERIAL.print(ts);
          DEBUG_SERIAL.print(F(" ms  enc="));
          DEBUG_SERIAL.println(enc, 1);
        }
      } else {
        still_count = 0;
      }
    }
    prev_enc = enc;

    printCSVRow(ts, enc, dx, dy, dz, t1_detected, t2_detected, t1_ms, t2_ms);
  }

  // Release
  moveToOpen();
  delay(800);
}

// ═══════════════════════════════════════════════════════════════════
//  SENSOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void initSensors() {
  DEBUG_SERIAL.println(F("# Initialising sensors..."));

  // Channel 0 — Strip 1 (Index + Thumb)
  tcaselect(0);
  mlx_idx1.begin(0, 0); mlx_idx1.setOverSampling(0); mlx_idx1.setDigitalFiltering(0);
  mlx_idx2.begin(0, 1); mlx_idx2.setOverSampling(0); mlx_idx2.setDigitalFiltering(0);
  mlx_thb1.begin(1, 0); mlx_thb1.setOverSampling(0); mlx_thb1.setDigitalFiltering(0);
  mlx_thb2.begin(1, 1); mlx_thb2.setOverSampling(0); mlx_thb2.setDigitalFiltering(0);

  // Channel 1 — Strip 2 (Middle)
  tcaselect(1);
  mlx_mid1.begin(0, 0); mlx_mid1.setOverSampling(0); mlx_mid1.setDigitalFiltering(0);
  mlx_mid2.begin(0, 1); mlx_mid2.setOverSampling(0); mlx_mid2.setDigitalFiltering(0);

  delay(100);
  DEBUG_SERIAL.println(F("# Sensors OK."));
}

// Average BASELINE_SAMPLES readings per sensor with fingers open, no object
void calibrateBaseline() {
  DEBUG_SERIAL.println(F("# Calibrating baseline — fingers open, no object..."));
  float sumX[NUM_SENSORS] = {}, sumY[NUM_SENSORS] = {}, sumZ[NUM_SENSORS] = {};

  for (int s = 0; s < BASELINE_SAMPLES; s++) {
    float dx[NUM_SENSORS], dy[NUM_SENSORS], dz[NUM_SENSORS];

    // Temporarily use dx/dy/dz arrays as raw buffers here
    tcaselect(0);
    mlx_idx1.readData(data); sumX[0]+=data.x; sumY[0]+=data.y; sumZ[0]+=data.z;
    mlx_idx2.readData(data); sumX[1]+=data.x; sumY[1]+=data.y; sumZ[1]+=data.z;
    mlx_thb1.readData(data); sumX[2]+=data.x; sumY[2]+=data.y; sumZ[2]+=data.z;
    mlx_thb2.readData(data); sumX[3]+=data.x; sumY[3]+=data.y; sumZ[3]+=data.z;

    tcaselect(1);
    mlx_mid1.readData(data); sumX[4]+=data.x; sumY[4]+=data.y; sumZ[4]+=data.z;
    mlx_mid2.readData(data); sumX[5]+=data.x; sumY[5]+=data.y; sumZ[5]+=data.z;

    delay(SAMPLE_INTERVAL_MS);
  }

  const char* names[NUM_SENSORS] = {"Idx1","Idx2","Thb1","Thb2","Mid1","Mid2"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    baseline[i][0] = sumX[i] / BASELINE_SAMPLES;
    baseline[i][1] = sumY[i] / BASELINE_SAMPLES;
    baseline[i][2] = sumZ[i] / BASELINE_SAMPLES;
    DEBUG_SERIAL.print(F("# Baseline ")); DEBUG_SERIAL.print(names[i]);
    DEBUG_SERIAL.print(F(" X=")); DEBUG_SERIAL.print(baseline[i][0],1);
    DEBUG_SERIAL.print(F(" Y=")); DEBUG_SERIAL.print(baseline[i][1],1);
    DEBUG_SERIAL.print(F(" Z=")); DEBUG_SERIAL.println(baseline[i][2],1);
  }
  DEBUG_SERIAL.println(F("# Baseline done."));
}

// Read all 6 sensors, return deltas from baseline
void readAllSensors(float dx[], float dy[], float dz[]) {
  tcaselect(0);
  mlx_idx1.readData(data); dx[0]=data.x-baseline[0][0]; dy[0]=data.y-baseline[0][1]; dz[0]=data.z-baseline[0][2];
  mlx_idx2.readData(data); dx[1]=data.x-baseline[1][0]; dy[1]=data.y-baseline[1][1]; dz[1]=data.z-baseline[1][2];
  mlx_thb1.readData(data); dx[2]=data.x-baseline[2][0]; dy[2]=data.y-baseline[2][1]; dz[2]=data.z-baseline[2][2];
  mlx_thb2.readData(data); dx[3]=data.x-baseline[3][0]; dy[3]=data.y-baseline[3][1]; dz[3]=data.z-baseline[3][2];

  tcaselect(1);
  mlx_mid1.readData(data); dx[4]=data.x-baseline[4][0]; dy[4]=data.y-baseline[4][1]; dz[4]=data.z-baseline[4][2];
  mlx_mid2.readData(data); dx[5]=data.x-baseline[5][0]; dy[5]=data.y-baseline[5][1]; dz[5]=data.z-baseline[5][2];
}

// ═══════════════════════════════════════════════════════════════════
//  MOTOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void initMotor() {
  DEBUG_SERIAL.println(F("# Initialising motor..."));
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  if (!dxl.ping(MOTOR_ID)) {
    DEBUG_SERIAL.println(F("# ERROR: XM430 not found! Check cable and ID in Dynamixel Wizard."));
    while (true) delay(1000); // Halt — cannot proceed without motor
  }

  dxl.torqueOff(MOTOR_ID);
  dxl.setOperatingMode(MOTOR_ID, OP_POSITION);
  // Profile velocity: XM430 unit = 0.229 rpm
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

// ═══════════════════════════════════════════════════════════════════
//  CSV OUTPUT
// ═══════════════════════════════════════════════════════════════════

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
                 bool t1, bool t2, uint32_t t1t, uint32_t t2t) {
  DEBUG_SERIAL.print(grasp_id);      DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(object_label);  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(ts);            DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(enc, 2);        DEBUG_SERIAL.print(',');

  for (int i = 0; i < NUM_SENSORS; i++) {
    DEBUG_SERIAL.print((int)dx[i]); DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print((int)dy[i]); DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print((int)dz[i]); DEBUG_SERIAL.print(',');
  }

  DEBUG_SERIAL.print(t1 ? 1 : 0); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(t2 ? 1 : 0); DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(t1t);         DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.println(t2t);
}

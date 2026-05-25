/*
 * ================================================================
 *  MLX90393 4-SENSOR STRIP TEST  -  Seeed XIAO ESP32S3
 *  File: sensor_test.ino
 * ================================================================
 *
 *  Purpose
 *  -------
 *  Bench-test the 4-sensor MLX90393 strip on the Seeed XIAO ESP32S3.
 *  Verifies all four chips respond, applies the field-tuned amp
 *  settings from the working bench reference code, and streams
 *  zero-referenced X/Y/Z deltas to the Arduino IDE Serial Plotter.
 *
 *  This is the bench / debug sketch. The production sketch
 *  (sensor_stream.ino, same parent folder) streams RAW values to the
 *  Python orchestrator instead of zero-referenced deltas, and runs
 *  an I2C recovery sequence at boot. Both sketches share the same
 *  amp settings.
 *
 *  Wiring  (XIAO ESP32S3  ->  sensor strip)
 *  ----------------------------------------------------------------
 *      3V3   --------->  VDD     MUST be 3.3V (chip abs max 3.6V)
 *      GND   --------->  GND     mate this first when connecting
 *      D4    --------->  SDA     default SDA on XIAO ESP32S3
 *      D5    --------->  SCL     default SCL on XIAO ESP32S3
 *
 *  External I2C pull-ups: 4.9 kOhm SDA->3V3 and SCL->3V3 are
 *  installed on the current rig. Good practice; gives margin
 *  beyond the ESP32S3's weak internal pull-ups.
 *
 *  No TCA9548A multiplexer is used. All four sensors sit on one I2C
 *  bus at unique addresses, set by hardware A1/A0 pins on each chip:
 *      ID1 = 0x14   A1=0  A0=0   begin(0, 0)
 *      ID2 = 0x15   A1=0  A0=1   begin(0, 1)
 *      ID3 = 0x16   A1=1  A0=0   begin(1, 0)
 *      ID4 = 0x17   A1=1  A0=1   begin(1, 1)
 *
 *  Safety checklist before powering up
 *  -----------------------------------
 *  1. Multimeter check: VDD-to-GND on the strip must read open
 *     (> 10 kOhm). A previous strip failed at 131 ohm after damage;
 *     never connect a strip that reads low resistance.
 *  2. Power the Nano OFF, plug the strip in (GND first), then power
 *     ON. Do not hot-plug - that has destroyed a strip before.
 *  3. The Nano's "3V3" pin must read 3.28 to 3.32 V before connecting.
 *     If it reads 5 V you are on the wrong pin and you will destroy
 *     the strip.
 *
 *  Usage
 *  -----
 *  1. Flash this sketch with the Arduino IDE (board: Arduino Nano 33 BLE).
 *  2. Open Tools -> Serial Plotter, set baud to 115200.
 *  3. Wait for the startup messages and "Streaming x/y/z deltas now".
 *  4. With nothing touching the strip, all 12 traces should sit
 *     near 0 with small jitter.
 *  5. Press the silicone pad over each sensor in turn. That sensor's
 *     three traces (mostly the Z trace) should jump and then return
 *     to ~0 when you release.
 *  6. Send 'r' (or 'R') over serial at any time to re-zero the
 *     baseline. Use this whenever you reposition the strip.
 *
 *  Onboard LED behaviour
 *  ---------------------
 *      slow blink (0.5 s)  -> alive and streaming
 *      fast blink (0.1 s)  -> fatal error, no sensors detected
 *
 *  Sensor amplification settings  (from the working reference code)
 *  ----------------------------------------------------------------
 *      setGainSel(0)              highest gain (5x XY / 8x Z)
 *      setResolution(2, 2, 2)     coarse LSB, no saturation at high field
 *      setOverSampling(2)         4x ADC averaging
 *      setDigitalFiltering(5)     heavier on-chip IIR
 *
 *  These differ from earlier (CONTEXT.md) settings that targeted
 *  finest LSB; the new settings prioritise head-room so the rotor
 *  field of the Dynamixel does not saturate the ADC once we are back
 *  on the hand. These settings MUST be kept byte-identical across
 *  this sketch and any future data-collection / inference firmware,
 *  or the trained model will see a different feature distribution
 *  at run time.
 *
 *  Required library
 *  ----------------
 *  arduino-MLX90393 by Theodore Yapo
 *  (https://github.com/tedyapo/arduino-MLX90393)
 * ================================================================
 */

#include <Wire.h>
#include "MLX90393.h"

// ================================================================
// CONFIG
// ================================================================
const uint32_t BAUD_RATE          = 115200;
const uint32_t SAMPLE_INTERVAL_MS = 20;    // 50 Hz update rate
const int      BASELINE_SAMPLES   = 50;    // ~1 second of averaging
const int      NUM_SENSORS        = 4;

// Order matches Data[] / DataR[] in the reference code:
//   index 0 -> 0x14 (ID1)
//   index 1 -> 0x15 (ID2)
//   index 2 -> 0x16 (ID3)
//   index 3 -> 0x17 (ID4)
const uint8_t  SENSOR_ADDR[NUM_SENSORS]  = { 0x14, 0x15, 0x16, 0x17 };
const uint8_t  SENSOR_A1[NUM_SENSORS]    = {    0,    0,    1,    1 };
const uint8_t  SENSOR_A0[NUM_SENSORS]    = {    0,    1,    0,    1 };
const char*    SENSOR_NAME[NUM_SENSORS]  = { "s0", "s1", "s2", "s3" };

// ================================================================
// STATE
// ================================================================
MLX90393        mlx[NUM_SENSORS];
bool            sensor_present[NUM_SENSORS] = { false, false, false, false };

// Same role as Data[] in the reference code: per-sensor per-axis baseline.
// Indexed as baseline[sensor][axis] where axis 0=X, 1=Y, 2=Z.
float           baseline[NUM_SENSORS][3];

// Per-loop scratch read buffer. Same role as DataR[] in the reference code,
// but kept in a single txyz struct since we only need the current sensor's
// reading at a time when iterating.
MLX90393::txyz  data;

uint32_t        next_sample_ms = 0;

// ================================================================
// HELPERS
// ================================================================

// Quick I2C probe used during startup so a single dead chip does not
// hang the rest of the sketch.
bool addrResponds(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Field-tuned amp settings. MUST stay byte-identical in any future
// firmware (data collection, inference) that uses this same strip.
void applyAmpSettings(MLX90393& s) {
  s.setGainSel(0);
  s.setResolution(2, 2, 2);
  s.setOverSampling(2);
  s.setDigitalFiltering(5);
}

// Average BASELINE_SAMPLES readings per sensor and store the result.
// Missing sensors get zero baselines and will report dx=dy=dz=0 forever.
void captureBaseline() {
  Serial.println(F("# Capturing baseline... keep the strip undisturbed."));
  float sumX[NUM_SENSORS] = { 0 };
  float sumY[NUM_SENSORS] = { 0 };
  float sumZ[NUM_SENSORS] = { 0 };
  int   cnt[NUM_SENSORS]  = { 0 };

  for (int n = 0; n < BASELINE_SAMPLES; n++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!sensor_present[i]) continue;
      if (mlx[i].readData(data) == MLX90393::STATUS_OK) {
        sumX[i] += data.x;
        sumY[i] += data.y;
        sumZ[i] += data.z;
        cnt[i]++;
      }
    }
    delay(SAMPLE_INTERVAL_MS);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensor_present[i] || cnt[i] == 0) {
      baseline[i][0] = baseline[i][1] = baseline[i][2] = 0;
      continue;
    }
    baseline[i][0] = sumX[i] / cnt[i];
    baseline[i][1] = sumY[i] / cnt[i];
    baseline[i][2] = sumZ[i] / cnt[i];
    Serial.print(F("# Baseline "));
    Serial.print(SENSOR_NAME[i]);
    Serial.print(F("  X=")); Serial.print(baseline[i][0], 0);
    Serial.print(F("  Y=")); Serial.print(baseline[i][1], 0);
    Serial.print(F("  Z=")); Serial.println(baseline[i][2], 0);
  }
  Serial.println(F("# Baseline done. Press a pad to see deflection."));
}

// Fast-blink the onboard LED forever to flag a fatal startup error.
void blinkErrorForever() {
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(BAUD_RATE);
  uint32_t serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start) < 4000) { /* wait for USB */ }

  Serial.println(F("# ============================================="));
  Serial.println(F("# MLX90393 4-sensor test  -  Arduino Nano 33 BLE"));
  Serial.println(F("# Direct I2C, no TCA9548A.  Expected: 0x14 0x15 0x16 0x17"));
  Serial.println(F("# ============================================="));
  Serial.println(F("# Open Tools -> Serial Plotter to visualise."));
  Serial.println(F("# Send 'r' over serial to re-zero baseline."));

  Wire.begin();
  delay(150);  // MLX90393 power-up + bus settle

  // ---- I2C scan over the 4 expected addresses ----
  Serial.print(F("# Scanning I2C bus:"));
  int found = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (addrResponds(SENSOR_ADDR[i])) {
      sensor_present[i] = true;
      found++;
      Serial.print(F(" 0x"));
      Serial.print(SENSOR_ADDR[i], HEX);
    }
  }
  Serial.print(F("   ("));
  Serial.print(found);
  Serial.println(F(" of 4 found)"));

  if (found == 0) {
    Serial.println(F("# ERROR: no MLX90393 detected."));
    Serial.println(F("# Power off. Confirm VDD=3.3V (NOT 5V), GND solid,"));
    Serial.println(F("# SDA on A4, SCL on A5. Then power on again."));
    blinkErrorForever();
  }

  // ---- Initialise each present sensor with the working amp settings ----
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensor_present[i]) {
      Serial.print(F("#   "));
      Serial.print(SENSOR_NAME[i]);
      Serial.print(F(" (0x"));
      Serial.print(SENSOR_ADDR[i], HEX);
      Serial.println(F(") NOT FOUND  -- output columns will be 0"));
      continue;
    }
    uint8_t st = mlx[i].begin(SENSOR_A1[i], SENSOR_A0[i]);
    applyAmpSettings(mlx[i]);
    Serial.print(F("#   "));
    Serial.print(SENSOR_NAME[i]);
    Serial.print(F(" (0x"));
    Serial.print(SENSOR_ADDR[i], HEX);
    Serial.print(F(")  begin status: 0x"));
    Serial.print(st, HEX);
    if (st != MLX90393::STATUS_OK) {
      Serial.print(F("  (warning - readings may be unreliable)"));
    }
    Serial.println();
  }

  delay(100);
  captureBaseline();
  Serial.println(F("# Streaming x/y/z deltas now at 50 Hz."));
  next_sample_ms = millis();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  // ---- Handle re-zero command from user ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R' || c == 'b' || c == 'B') {
      captureBaseline();
      next_sample_ms = millis();
    }
  }

  // ---- Slow blink for "alive" indication ----
  uint32_t now = millis();
  digitalWrite(LED_BUILTIN, (now / 500) % 2);

  // ---- Pace loop to ~50 Hz ----
  if ((int32_t)(now - next_sample_ms) < 0) return;
  next_sample_ms += SAMPLE_INTERVAL_MS;

  // ---- Read each sensor and print zero-referenced deltas ----
  // Output format is Serial Plotter compatible:
  //   "label:value label:value ..."
  // The Arduino IDE Plotter uses the labels as the legend.
  bool first = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float dx = 0, dy = 0, dz = 0;
    if (sensor_present[i]) {
      if (mlx[i].readData(data) == MLX90393::STATUS_OK) {
        dx = data.x - baseline[i][0];
        dy = data.y - baseline[i][1];
        dz = data.z - baseline[i][2];
      }
    }
    if (!first) Serial.print(' ');
    first = false;
    Serial.print(SENSOR_NAME[i]); Serial.print(F("_x:")); Serial.print(dx, 0);
    Serial.print(' ');
    Serial.print(SENSOR_NAME[i]); Serial.print(F("_y:")); Serial.print(dy, 0);
    Serial.print(' ');
    Serial.print(SENSOR_NAME[i]); Serial.print(F("_z:")); Serial.print(dz, 0);
  }
  Serial.println();
}

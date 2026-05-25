/*
 * ================================================================
 *  MLX90393 SENSOR STREAM  -  Seeed XIAO ESP32S3
 *  File: sensor_stream.ino
 * ================================================================
 *
 *  Role
 *  ----
 *  Production sensor firmware for the split-architecture prosthetic
 *  hand. Pure sensor pump: reads 4× MLX90393 at 50 Hz and streams
 *  raw X/Y/Z values over USB serial to the Python orchestrator.
 *  No decisions (no thresholding, no motor-field subtraction, no
 *  feature extraction). Python does all of that.
 *
 *  Wiring (XIAO ESP32S3 -> sensor strip)
 *  -------------------------------------
 *      3V3  -> VDD       3.3 V only (chip abs max 3.6 V)
 *      GND  -> GND       mate first
 *      D4   -> SDA       default I2C SDA on XIAO ESP32S3
 *      D5   -> SCL       default I2C SCL on XIAO ESP32S3
 *      4.9 kOhm pull-ups: SDA->3V3 and SCL->3V3
 *
 *  Sensor addresses (no TCA9548A, all four on one bus):
 *      0x14  ID1   A1=0 A0=0   begin(0,0)
 *      0x15  ID2   A1=0 A0=1   begin(0,1)
 *      0x16  ID3   A1=1 A0=0   begin(1,0)
 *      0x17  ID4   A1=1 A0=1   begin(1,1)
 *
 *  I2C recovery sequence
 *  ---------------------
 *  At boot we try up to MAX_INIT_ATTEMPTS to see all 4 sensors:
 *    Attempt 1: Wire.begin() default 100 kHz, scan + init.
 *    If <4 found:
 *      - Wire.end() to release the bus.
 *      - Clock SCL manually 16 times with SDA released to free any
 *        stuck slave that may be holding SDA low.
 *      - delay 50 ms.
 *      - Wire.begin() again, retry scan + init.
 *  After MAX_INIT_ATTEMPTS, if we still don't have all 4, we report
 *  which addresses are missing on serial and refuse to enter the
 *  streaming state. The Python orchestrator polls for ENUM_READY
 *  and aborts the pipeline if it never arrives.
 *
 *  Serial protocol
 *  ---------------
 *  Commands from Python (one ASCII char per command, terminated by
 *  \n or end of buffer):
 *    p   -> reply "# pong"
 *    e   -> reply "# ENUM <n_found> 0x14 0x15 0x16 0x17"  (only present ones listed)
 *    s   -> start streaming (replies "# STREAM_ON")
 *    x   -> stop  streaming (replies "# STREAM_OFF")
 *    r   -> recovery: re-run the I2C recovery sequence and re-init
 *
 *  Stream lines (only emitted while streaming, one per 20 ms):
 *    S,<millis>,<s0x>,<s0y>,<s0z>,<s1x>,<s1y>,<s1z>,
 *      <s2x>,<s2y>,<s2z>,<s3x>,<s3y>,<s3z>
 *
 *  Status / event lines (any time):
 *    # ENUM_READY   <when all 4 sensors are initialised>
 *    # ENUM_FAIL    <when recovery exhausted; lists missing addresses>
 *    # STREAM_ON / # STREAM_OFF
 *    # ERROR <description>
 *
 *  Amp settings (must match every other sketch / pipeline stage)
 *  -------------------------------------------------------------
 *      setGainSel(0)              highest gain (5x XY / 8x Z)
 *      setResolution(2, 2, 2)     coarse LSB, no saturation
 *      setOverSampling(2)         4x ADC averaging
 *      setDigitalFiltering(5)     heavier on-chip IIR
 *
 *  Required library: arduino-MLX90393 by Theodore Yapo.
 * ================================================================
 */

#include <Wire.h>
#include "MLX90393.h"

// ================================================================
// CONFIG
// ================================================================
const uint32_t BAUD_RATE          = 115200;
const uint32_t SAMPLE_INTERVAL_MS = 20;     // 50 Hz
const int      NUM_SENSORS        = 4;
const int      MAX_INIT_ATTEMPTS  = 3;
const int      I2C_SDA_PIN        = D4;
const int      I2C_SCL_PIN        = D5;

const uint8_t  SENSOR_ADDR[NUM_SENSORS] = { 0x14, 0x15, 0x16, 0x17 };
const uint8_t  SENSOR_A1[NUM_SENSORS]   = {    0,    0,    1,    1 };
const uint8_t  SENSOR_A0[NUM_SENSORS]   = {    0,    1,    0,    1 };

// ================================================================
// STATE
// ================================================================
MLX90393        mlx[NUM_SENSORS];
bool            sensor_present[NUM_SENSORS] = { false, false, false, false };
bool            enum_ready                  = false;
bool            streaming                   = false;
uint32_t        next_sample_ms              = 0;
MLX90393::txyz  read_buf;

// ================================================================
// HELPERS
// ================================================================
bool addrResponds(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void applyAmpSettings(MLX90393& s) {
  s.setGainSel(0);
  s.setResolution(2, 2, 2);
  s.setOverSampling(2);
  s.setDigitalFiltering(5);
}

// Manual SCL toggle to unstick any I2C slave holding SDA low.
// Standard trick before re-attempting Wire.begin().
void recoverBus() {
  Wire.end();
  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 16; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delay(20);
}

// Single attempt: open bus, scan addresses, init each present chip.
// Returns number of sensors that responded.
int tryInitAll() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  delay(150);  // MLX90393 power-up settle

  int found = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_present[i] = false;
    if (addrResponds(SENSOR_ADDR[i])) {
      uint8_t st = mlx[i].begin(SENSOR_A1[i], SENSOR_A0[i]);
      applyAmpSettings(mlx[i]);
      if (st == MLX90393::STATUS_OK) {
        sensor_present[i] = true;
        found++;
      } else {
        Serial.print(F("# ERROR sensor 0x"));
        Serial.print(SENSOR_ADDR[i], HEX);
        Serial.print(F(" begin status=0x"));
        Serial.println(st, HEX);
      }
    }
  }
  return found;
}

void reportMissing() {
  Serial.print(F("# ENUM_FAIL missing:"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensor_present[i]) {
      Serial.print(F(" 0x"));
      Serial.print(SENSOR_ADDR[i], HEX);
    }
  }
  Serial.println();
}

void reportEnum() {
  int n = 0;
  for (int i = 0; i < NUM_SENSORS; i++) if (sensor_present[i]) n++;
  Serial.print(F("# ENUM "));
  Serial.print(n);
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensor_present[i]) {
      Serial.print(F(" 0x"));
      Serial.print(SENSOR_ADDR[i], HEX);
    }
  }
  Serial.println();
}

// Full init flow: try -> recover -> retry, up to MAX_INIT_ATTEMPTS.
// Sets enum_ready true iff all 4 sensors are present.
void initWithRecovery() {
  enum_ready = false;
  for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
    Serial.print(F("# init attempt "));
    Serial.print(attempt);
    Serial.print(F("/"));
    Serial.println(MAX_INIT_ATTEMPTS);

    int found = tryInitAll();
    if (found == NUM_SENSORS) {
      enum_ready = true;
      reportEnum();
      Serial.println(F("# ENUM_READY"));
      return;
    }

    Serial.print(F("# only "));
    Serial.print(found);
    Serial.println(F(" of 4 found, running bus recovery"));
    if (attempt < MAX_INIT_ATTEMPTS) {
      recoverBus();
    }
  }
  reportEnum();
  reportMissing();
  Serial.println(F("# Streaming disabled until recovery succeeds (send 'r' to retry)."));
}

void emitSample() {
  uint32_t now = millis();
  Serial.print('S');
  Serial.print(',');
  Serial.print(now);
  for (int i = 0; i < NUM_SENSORS; i++) {
    float x = 0, y = 0, z = 0;
    if (sensor_present[i]) {
      if (mlx[i].readData(read_buf) == MLX90393::STATUS_OK) {
        x = read_buf.x;
        y = read_buf.y;
        z = read_buf.z;
      }
    }
    Serial.print(','); Serial.print(x, 0);
    Serial.print(','); Serial.print(y, 0);
    Serial.print(','); Serial.print(z, 0);
  }
  Serial.println();
}

void handleCommand(char c) {
  switch (c) {
    case 'p':
      Serial.println(F("# pong"));
      break;
    case 'e':
      reportEnum();
      break;
    case 's':
      if (!enum_ready) {
        Serial.println(F("# ERROR cannot stream, ENUM_FAIL — send 'r' to retry init"));
        return;
      }
      streaming      = true;
      next_sample_ms = millis();
      Serial.println(F("# STREAM_ON"));
      break;
    case 'x':
      streaming = false;
      Serial.println(F("# STREAM_OFF"));
      break;
    case 'r':
      streaming = false;
      Serial.println(F("# recovery requested"));
      initWithRecovery();
      break;
    case '\n':
    case '\r':
    case ' ':
      break;  // ignore whitespace
    default:
      Serial.print(F("# ERROR unknown command '"));
      Serial.print(c);
      Serial.println(F("'"));
      break;
  }
}

// ================================================================
// SETUP / LOOP
// ================================================================
void setup() {
  Serial.begin(BAUD_RATE);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) {}

  Serial.println(F("# ============================================="));
  Serial.println(F("# sensor_stream.ino  -  XIAO ESP32S3"));
  Serial.println(F("# 4x MLX90393 @ 0x14/15/16/17, direct I2C on D4/D5"));
  Serial.println(F("# ============================================="));

  initWithRecovery();
}

void loop() {
  // Drain serial commands.
  while (Serial.available()) {
    handleCommand((char)Serial.read());
  }

  // Stream paced to 50 Hz.
  if (!streaming) return;
  uint32_t now = millis();
  if ((int32_t)(now - next_sample_ms) < 0) return;
  next_sample_ms += SAMPLE_INTERVAL_MS;
  emitSample();
}

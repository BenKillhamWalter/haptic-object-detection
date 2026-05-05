#include <MLX90393.h>

#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

#define TCA_ADDR      0x70

void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial Monitor to connect
  delay(1000);      // Extra buffer

  Wire.begin();
  for (uint8_t ch = 0; ch < 3; ch++) {
    tcaselect(ch);
    Serial.print("Channel "); Serial.println(ch);
    for (uint8_t addr = 8; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("  Found 0x"); Serial.println(addr, HEX);
      }
    }
  }
}
void loop() {}

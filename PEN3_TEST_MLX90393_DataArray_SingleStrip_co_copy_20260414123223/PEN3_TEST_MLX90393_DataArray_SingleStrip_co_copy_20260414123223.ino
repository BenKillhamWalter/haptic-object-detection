/////////////////// three sensors
#include <Wire.h>
#include "MLX90393.h" // From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo

MLX90393 mlx0;
MLX90393 mlx1;
MLX90393 mlx2;
MLX90393::txyz data; // Create a structure of four floats (t, x, y, and z)

float DataR[9];
float Data[9];

// Define the size of the moving average filter
const int filterSize = 20;

// Initialize an array to store past readings
float filterArray[filterSize];

void setup() {
  Serial.begin(115200);

  Wire.begin();
  mlx0.begin(0, 0);
  mlx1.begin(0, 1);
  // mlx2.begin(1, 1);

  mlx0.setOverSampling(0);
  mlx1.setOverSampling(0);
  // mlx2.setOverSampling(0);

  mlx0.setDigitalFiltering(0);
  mlx1.setDigitalFiltering(0);
  // mlx2.setDigitalFiltering(0);

  delay(100);

  tcaselect(0); // 0 - 7 of the TCA9548A channels

  // First baseline read
  mlx0.readData(data);
  Data[0] = data.x; Data[1] = data.y; Data[2] = data.z;

  mlx1.readData(data);
  Data[3] = data.x; Data[4] = data.y; Data[5] = data.z;

  // lx2.readData(data);
  // Data[6] = data.x; Data[7] = data.y; Data[8] = data.z;

  delay(300);

  // Second baseline read
  mlx0.readData(data);
  Data[0] = data.x; Data[1] = data.y; Data[2] = data.z;

  mlx1.readData(data);
  Data[3] = data.x; Data[4] = data.y; Data[5] = data.z;

  // mlx2.readData(data);
  // Data[6] = data.x; Data[7] = data.y; Data[8] = data.z;
}

void loop() {
  tcaselect(0); // 0 - 7 of the TCA9548A channels

  mlx0.readData(data);
  DataR[0] = data.x; DataR[1] = data.y; DataR[2] = data.z;

  mlx1.readData(data);
  DataR[3] = data.x; DataR[4] = data.y; DataR[5] = data.z;

  // mlx2.readData(data);
  // DataR[6] = data.x; DataR[7] = data.y; DataR[8] = data.z;

  float v0 = DataR[2] - Data[2];
  if (isnan(v0) || isinf(v0)) {
    Serial.println("v0 invalid");
  }

  // Print 9 values: x,y,z for 3 sensors
  Serial.print(DataR[0] - Data[0], 0);
  Serial.print(",");
  Serial.print(DataR[1] - Data[1], 0);
  Serial.print(",");
  Serial.print(DataR[2] - Data[2], 0);
  Serial.print(",");

  Serial.print(DataR[3] - Data[3], 0);
  Serial.print(",");
  Serial.print(DataR[4] - Data[4], 0);
  Serial.print(",");
  Serial.println(DataR[5] - Data[5], 0);
  // Serial.print(",");

  // Serial.print(DataR[6] - Data[6], 0);
  // Serial.print(",");
  // Serial.print(DataR[7] - Data[7], 0);
  // Serial.print(",");
  // Serial.println(DataR[8] - Data[8], 0);

  // Magnitude for sensor 3
  // float diff1 = DataR[6] - Data[6];
  // float diff2 = DataR[7] - Data[7];
  // float diff3 = DataR[8] - Data[8];

  // float sumOfSquares = diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
  // float result = sqrt(sumOfSquares);

  // Read new data and update the filter
  // float filteredValue = movingAverage(DataR[8] - Data[8]);

  delay(100);
}

// Initialize I2C buses using TCA9548A I2C Multiplexer
void tcaselect(uint8_t i2c_bus) {
  if (i2c_bus > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i2c_bus);
  Wire.endTransmission();
}

// Function to add a new reading to the filter array and return the average
float movingAverage(float newValue) {
  for (int i = filterSize - 1; i > 0; i--) {
    filterArray[i] = filterArray[i - 1];
  }

  filterArray[0] = newValue;

  float sum = 0;
  for (int i = 0; i < filterSize; i++) {
    sum += filterArray[i];
  }
  return sum / filterSize;
}
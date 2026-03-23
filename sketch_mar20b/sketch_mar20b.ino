// One way to calibaret acc
#include <Wire.h>

#define I2C_SDA 11
#define I2C_SCL 10
const uint8_t MPU_ADDR = 0x68; 

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  // Set Accel to ±4g (1g = 8192 raw)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); 
  Wire.write(0x08); 
  Wire.endTransmission(true);

  Serial.println("Calculating Offsets... DO NOT MOVE THE BOARD.");
  delay(2000); // Give you time to let go of the board

  int32_t sumX = 0, sumY = 0, sumZ = 0;
  const int num_samples = 2000;

  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    
    sumX += (Wire.read() << 8 | Wire.read());
    sumY += (Wire.read() << 8 | Wire.read());
    sumZ += (Wire.read() << 8 | Wire.read());
    delay(3);
  }

  // Calculate Averages
  float avgX = sumX / (float)num_samples;
  float avgY = sumY / (float)num_samples;
  float avgZ = sumZ / (float)num_samples;

  // Calculate Offsets
  // Target for X and Y is 0. Target for Z is 1g (8192 on the ±4g scale).
  int16_t offsetX = 0 - avgX;
  int16_t offsetY = 0 - avgY;
  int16_t offsetZ = 8192 - avgZ;

  Serial.println("--- HARDCODE THESE INTO YOUR MAIN FLIGHT LOOP ---");
  Serial.printf("const int16_t ACCEL_OFFSET_X = %d;\n", offsetX);
  Serial.printf("const int16_t ACCEL_OFFSET_Y = %d;\n", offsetY);
  Serial.printf("const int16_t ACCEL_OFFSET_Z = %d;\n", offsetZ);
}

void loop() {
  // Nothing to do here
}
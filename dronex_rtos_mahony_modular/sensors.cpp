#include "sensors.h"

#define MPU_FILTER_CONFIG 0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define I2C_SDA 11
#define I2C_SCL 10

const uint8_t MPU_ADDR = 0x68;
const int16_t ACCEL_OFFSET_X = 1000;
const int16_t ACCEL_OFFSET_Y = -700;
const int16_t ACCEL_OFFSET_Z = 1160;

float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;

void Sensors_Init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Wake up
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_FILTER_CONFIG); // DLPF
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CONFIG); // Gyro +/- 500 deg/s
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CONFIG); // Accel +/- 4g
  Wire.write(0x08);
  Wire.endTransmission();
}

void Sensors_Calibrate() {
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    
    int16_t gX = (Wire.read() << 8 | Wire.read());
    int16_t gY = (Wire.read() << 8 | Wire.read());
    int16_t gZ = (Wire.read() << 8 | Wire.read());
    
    gyroErrorX += (gX / 65.5); 
    gyroErrorY += (gY / 65.5);
    gyroErrorZ += (gZ / 65.5);
    
    vTaskDelay(pdMS_TO_TICKS(3)); 
  }
  
  gyroErrorX /= 2000;
  gyroErrorY /= 2000;
  gyroErrorZ /= 2000;
}

void Sensors_Read(float &gyroRadX, float &gyroRadY, float &gyroRadZ, 
                  float &accX, float &accY, float &accZ, 
                  float &gyroRateX, float &gyroRateY, float &gyroRateZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t rawAccX = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
  int16_t rawAccY = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
  int16_t rawAccZ = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
  int16_t tempRaw = (Wire.read() << 8 | Wire.read()); // Ignored
  int16_t rawGyX  = (Wire.read() << 8 | Wire.read());
  int16_t rawGyY  = (Wire.read() << 8 | Wire.read());
  int16_t rawGyZ  = (Wire.read() << 8 | Wire.read());
  
  accX = -rawAccY;
  accY = -rawAccX;
  accZ = (float)rawAccZ;
  
  int16_t gyroX = -rawGyY;
  int16_t gyroY = -rawGyX;
  int16_t gyroZ = rawGyZ;

  gyroRateX = (gyroX / 65.5) - gyroErrorX;
  gyroRateY = (gyroY / 65.5) - gyroErrorY;
  gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;

  gyroRadX = gyroRateX * 0.0174533f;
  gyroRadY = gyroRateY * 0.0174533f;
  gyroRadZ = gyroRateZ * 0.0174533f;
}
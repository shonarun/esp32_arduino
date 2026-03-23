// This shall be used to test the IMU MPU 6050

#include <Wire.h>

#define I2C_SDA 11
#define I2C_SCL 10

const uint8_t MPU_ADDR = 0x68;
const int16_t ACCEL_OFFSET_X = -83;
const int16_t ACCEL_OFFSET_Y = -74;
const int16_t ACCEL_OFFSET_Z = -28;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;

void calibrateGyro() {
  // Read the gyro 2000 times to get a solid average
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Start of Gyro registers
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    
    int16_t gX = (Wire.read() << 8 | Wire.read());
    int16_t gY = (Wire.read() << 8 | Wire.read());
    int16_t gZ = (Wire.read() << 8 | Wire.read());
    
    // Convert to degrees/s and add to total
    gyroErrorX += (gX / 65.5); 
    gyroErrorY += (gY / 65.5);
    gyroErrorZ += (gZ / 65.5);
    delay(3); // Small delay to let sensor hardware keep up
  }
  
  // Divide by 2000 to get the average offset
  gyroErrorX /= 2000;
  gyroErrorY /= 2000;
  gyroErrorZ /= 2000;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

// Wake up the MPU6050 (It starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Write 0 to wake it up
  Wire.endTransmission(true);

  // Set Digital Low-Pass Filter (Register 0x1A)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x08); // 0x08 sets it to ±500°/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x08); // 0x08 sets it to ±4g
  Wire.endTransmission();

  calibrateGyro();

}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);

  // Burst read all 14 bytes (6 for Accel, 2 for Temp, 6 for Gyro)
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  
  // Shift the 8-bit registers into 16-bit integers
  accX    = (Wire.read() << 8 | Wire.read());
  accY    = (Wire.read() << 8 | Wire.read());
  accZ    = (Wire.read() << 8 | Wire.read());
  tempRaw = (Wire.read() << 8 | Wire.read());
  gyroX   = (Wire.read() << 8 | Wire.read());
  gyroY   = (Wire.read() << 8 | Wire.read());
  gyroZ   = (Wire.read() << 8 | Wire.read());

  accX += ACCEL_OFFSET_X;
  accY += ACCEL_OFFSET_Y;
  accZ += ACCEL_OFFSET_Z;

  float gyroRateX = (gyroX / 65.5) - gyroErrorX;
  float gyroRateY = (gyroY / 65.5) - gyroErrorY;
  float gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;
  float accAnglePitch = atan2( accY, sqrt((float)accX * accX + (float)accZ * accZ)) * 57.2958;
  float accAngleRoll  = atan2(-accX, sqrt((float)accY * accY + (float)accZ * accZ)) * 57.2958;

  Serial.printf("Pitch: %f, Roll: %f, GX: %f, GY: %f, GZ: %f, AZ: %d\n", accAnglePitch, accAngleRoll, gyroRateX, gyroRateY, gyroRateZ, accZ);
}

#include "estimator.h"
#include "config.h"
#include <Wire.h>

static const float Q_angle = 0.001f;   
static const float Q_bias = 0.003f;    
static const float R_measure = 0.03f;  

static float roll_bias = 0.0f;
static float P_roll[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
static float pitch_bias = 0.0f;
static float P_pitch[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

static float actual_pitch = 0.0f, actual_roll = 0.0f, actual_yaw = 0.0f;
static float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;

void initEstimator() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x04); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
}

void calibrateEstimator() {
  gyroErrorX = 0; gyroErrorY = 0; gyroErrorZ = 0;
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    gyroErrorX += ((Wire.read() << 8 | Wire.read()) / 65.5f); 
    gyroErrorY += ((Wire.read() << 8 | Wire.read()) / 65.5f);
    gyroErrorZ += ((Wire.read() << 8 | Wire.read()) / 65.5f);
    vTaskDelay(pdMS_TO_TICKS(3)); 
  }
  gyroErrorX /= 2000.0f; gyroErrorY /= 2000.0f; gyroErrorZ /= 2000.0f;
}

AttitudeData updateEstimator(float dt) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t rawAccX = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
  int16_t rawAccY = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
  int16_t rawAccZ = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
  Wire.read(); Wire.read(); 
  int16_t rawGyX  = (Wire.read() << 8 | Wire.read());
  int16_t rawGyY  = (Wire.read() << 8 | Wire.read());
  int16_t rawGyZ  = (Wire.read() << 8 | Wire.read());

  float accX = -rawAccY; float accY = -rawAccX; float accZ = rawAccZ;
  float gRateX = (-rawGyY / 65.5f) - gyroErrorX;
  float gRateY = (-rawGyX / 65.5f) - gyroErrorY;
  float gRateZ = (rawGyZ / 65.5f) - gyroErrorZ;

  float accRoll = atan2(accY, accZ) * 57.2958f;
  float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958f;

  float rate_roll = gRateX - roll_bias; actual_roll += dt * rate_roll;
  P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
  P_roll[0][1] -= dt * P_roll[1][1]; P_roll[1][0] -= dt * P_roll[1][1]; P_roll[1][1] += Q_bias * dt;
  float S_roll = P_roll[0][0] + R_measure;
  float K_roll[2] = {P_roll[0][0] / S_roll, P_roll[1][0] / S_roll};
  float y_roll = accRoll - actual_roll; actual_roll += K_roll[0] * y_roll; roll_bias += K_roll[1] * y_roll;
  float P00_temp_r = P_roll[0][0], P01_temp_r = P_roll[0][1];
  P_roll[0][0] -= K_roll[0] * P00_temp_r; P_roll[0][1] -= K_roll[0] * P01_temp_r;
  P_roll[1][0] -= K_roll[1] * P00_temp_r; P_roll[1][1] -= K_roll[1] * P01_temp_r;

  float rate_pitch = gRateY - pitch_bias; actual_pitch += dt * rate_pitch;
  P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
  P_pitch[0][1] -= dt * P_pitch[1][1]; P_pitch[1][0] -= dt * P_pitch[1][1]; P_pitch[1][1] += Q_bias * dt;
  float S_pitch = P_pitch[0][0] + R_measure;
  float K_pitch[2] = {P_pitch[0][0] / S_pitch, P_pitch[1][0] / S_pitch};
  float y_pitch = accPitch - actual_pitch; actual_pitch += K_pitch[0] * y_pitch; pitch_bias += K_pitch[1] * y_pitch;
  float P00_temp_p = P_pitch[0][0], P01_temp_p = P_pitch[0][1];
  P_pitch[0][0] -= K_pitch[0] * P00_temp_p; P_pitch[0][1] -= K_pitch[0] * P01_temp_p;
  P_pitch[1][0] -= K_pitch[1] * P00_temp_p; P_pitch[1][1] -= K_pitch[1] * P01_temp_p;

  actual_yaw += gRateZ * dt; 
  return {actual_pitch, actual_roll, actual_yaw, gRateX, gRateY, gRateZ};
}
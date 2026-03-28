#include "sensors.h"
#include "config.h"
#include <Wire.h>

#define MPU_FILTER_CONFIG 0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C

// Localized Kalman State (Hidden from the rest of the program)
static const float Q_angle = 0.001f;   
static const float Q_bias = 0.003f;    
static const float R_measure = 0.03f;  

static float roll_bias = 0.0f;
static float P_roll[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
static float pitch_bias = 0.0f;
static float P_pitch[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

void initMPU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

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

void calibrateGyro() {
  gyroErrorX = 0; gyroErrorY = 0; gyroErrorZ = 0;
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
  gyroErrorX /= 2000; gyroErrorY /= 2000; gyroErrorZ /= 2000;
}

void Kalman_Update(float gyroRateX, float gyroRateY, float gyroRateZ, float accX, float accY, float accZ, float dt) {
    float accRoll = atan2(accY, accZ) * 57.2958f;
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958f;

    // --- ROLL KALMAN FILTER ---
    float rate_roll = gyroRateX - roll_bias;
    actual_roll += dt * rate_roll;
    P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
    P_roll[0][1] -= dt * P_roll[1][1];
    P_roll[1][0] -= dt * P_roll[1][1];
    P_roll[1][1] += Q_bias * dt;
    float S_roll = P_roll[0][0] + R_measure;
    float K_roll[2] = {P_roll[0][0] / S_roll, P_roll[1][0] / S_roll};
    float y_roll = accRoll - actual_roll;
    actual_roll += K_roll[0] * y_roll;
    roll_bias += K_roll[1] * y_roll;
    float P00_temp_r = P_roll[0][0];
    float P01_temp_r = P_roll[0][1];
    P_roll[0][0] -= K_roll[0] * P00_temp_r;
    P_roll[0][1] -= K_roll[0] * P01_temp_r;
    P_roll[1][0] -= K_roll[1] * P00_temp_r;
    P_roll[1][1] -= K_roll[1] * P01_temp_r;

    // --- PITCH KALMAN FILTER ---
    float rate_pitch = gyroRateY - pitch_bias;
    actual_pitch += dt * rate_pitch;
    P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
    P_pitch[0][1] -= dt * P_pitch[1][1];
    P_pitch[1][0] -= dt * P_pitch[1][1];
    P_pitch[1][1] += Q_bias * dt;
    float S_pitch = P_pitch[0][0] + R_measure;
    float K_pitch[2] = {P_pitch[0][0] / S_pitch, P_pitch[1][0] / S_pitch};
    float y_pitch = accPitch - actual_pitch;
    actual_pitch += K_pitch[0] * y_pitch;
    pitch_bias += K_pitch[1] * y_pitch;
    float P00_temp_p = P_pitch[0][0];
    float P01_temp_p = P_pitch[0][1];
    P_pitch[0][0] -= K_pitch[0] * P00_temp_p;
    P_pitch[0][1] -= K_pitch[0] * P01_temp_p;
    P_pitch[1][0] -= K_pitch[1] * P00_temp_p;
    P_pitch[1][1] -= K_pitch[1] * P01_temp_p;

    // --- YAW UPDATE ---
    actual_yaw += gyroRateZ * dt; 
}
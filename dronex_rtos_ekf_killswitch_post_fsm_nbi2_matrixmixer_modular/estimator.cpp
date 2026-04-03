#include "estimator.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

RawSensorData latest_sensor_data = {0,0,0,0,0,0};
portMUX_TYPE sensor_mux = portMUX_INITIALIZER_UNLOCKED;

// --- EKF Variables ---
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
    
    gyroErrorX += ( (int16_t) (Wire.read() << 8 | Wire.read()) / 65.5f); 
    gyroErrorY += ( (int16_t) (Wire.read() << 8 | Wire.read()) / 65.5f);
    gyroErrorZ += ( (int16_t) (Wire.read() << 8 | Wire.read()) / 65.5f);
    vTaskDelay(pdMS_TO_TICKS(3)); 
  }
  gyroErrorX /= 2000.0f; 
  gyroErrorY /= 2000.0f; 
  gyroErrorZ /= 2000.0f;
}

// ====================================================
// DEDICATED I2C POLLING TASK (Non-blocking)
// ====================================================
void Task_SensorRead(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz polling

  for(;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    Wire.beginTransmission(MPU_ADDR); 
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

    if(Wire.available() >= 14) {
      RawSensorData tempData;
      tempData.rawAccX = (int16_t) (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
      tempData.rawAccY = (int16_t) (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
      tempData.rawAccZ = (int16_t) (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
      Wire.read(); Wire.read(); // Skip temp
      tempData.rawGyX  = (int16_t) (Wire.read() << 8 | Wire.read());
      tempData.rawGyY  = (int16_t) (Wire.read() << 8 | Wire.read());
      tempData.rawGyZ  = (int16_t) (Wire.read() << 8 | Wire.read());

      // Thread-safe write to global variable
      portENTER_CRITICAL(&sensor_mux);
      latest_sensor_data = tempData;
      portEXIT_CRITICAL(&sensor_mux);
    }
  }
}

// ====================================================
// EKF ESTIMATOR MATH
// ====================================================
AttitudeData updateEstimator(float dt) {
  RawSensorData currentData;

  // Thread-safe read from global variable (Non-blocking!)
  portENTER_CRITICAL(&sensor_mux);
  currentData = latest_sensor_data;
  portEXIT_CRITICAL(&sensor_mux);

  // Axis mapping
  float accX = -currentData.rawAccY; 
  float accY = -currentData.rawAccX; 
  float accZ = currentData.rawAccZ;
  
  float gRateX = (-currentData.rawGyY / 65.5f) - gyroErrorX;
  float gRateY = (-currentData.rawGyX / 65.5f) - gyroErrorY;
  float gRateZ = (currentData.rawGyZ / 65.5f) - gyroErrorZ;

  // Calculate accelerometer angles
  float accRoll = atan2(accY, accZ) * 57.2958f;
  float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958f;

  // --- Roll EKF ---
  float rate_roll = gRateX - roll_bias; 
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
  float P00_temp_r = P_roll[0][0], P01_temp_r = P_roll[0][1];
  P_roll[0][0] -= K_roll[0] * P00_temp_r; 
  P_roll[0][1] -= K_roll[0] * P01_temp_r;
  P_roll[1][0] -= K_roll[1] * P00_temp_r; 
  P_roll[1][1] -= K_roll[1] * P01_temp_r;

  // --- Pitch EKF ---
  float rate_pitch = gRateY - pitch_bias; 
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
  float P00_temp_p = P_pitch[0][0], P01_temp_p = P_pitch[0][1];
  P_pitch[0][0] -= K_pitch[0] * P00_temp_p; 
  P_pitch[0][1] -= K_pitch[0] * P01_temp_p;
  P_pitch[1][0] -= K_pitch[1] * P00_temp_p; 
  P_pitch[1][1] -= K_pitch[1] * P01_temp_p;

  // --- Yaw (Gyro Integration Only) ---
  actual_yaw += gRateZ * dt; 

  return {actual_pitch, actual_roll, actual_yaw, gRateX, gRateY, gRateZ};
}
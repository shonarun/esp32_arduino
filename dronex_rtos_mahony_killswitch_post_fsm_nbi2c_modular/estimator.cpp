#include "estimator.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

// Shared global data and FreeRTOS Spinlock (Mutex)
volatile RawSensorData latest_sensor_data = {0,0,0,0,0,0};
portMUX_TYPE sensor_mux = portMUX_INITIALIZER_UNLOCKED;

// Mahony Filter variables
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
static float twoKpDef = 0.8f;   
static float twoKiDef = 0.002f; 
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static float actual_pitch = 0.0f, actual_roll = 0.0f, actual_yaw = 0.0f;
static float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;

// Fast inverse square-root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void initEstimator() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);
  
  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  // DLPF (Digital Low Pass Filter)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x04); Wire.endTransmission();
  // Gyro +/- 500 deg/s
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  // Accel +/- 4g
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
  gyroErrorX /= 2000.0f; 
  gyroErrorY /= 2000.0f; 
  gyroErrorZ /= 2000.0f;
}

// ====================================================
// NEW: DEDICATED I2C POLLING TASK
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
      tempData.rawAccX = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
      tempData.rawAccY = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
      tempData.rawAccZ = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
      Wire.read(); Wire.read(); // Skip temp
      tempData.rawGyX  = (Wire.read() << 8 | Wire.read());
      tempData.rawGyY  = (Wire.read() << 8 | Wire.read());
      tempData.rawGyZ  = (Wire.read() << 8 | Wire.read());

      // Thread-safe write to global variable
      portENTER_CRITICAL(&sensor_mux);
      latest_sensor_data = tempData;
      portEXIT_CRITICAL(&sensor_mux);
    }
  }
}

AttitudeData updateEstimator(float dt) {
  RawSensorData currentData;

  // Thread-safe read from global variable (Non-blocking!)
  portENTER_CRITICAL(&sensor_mux);
  currentData = latest_sensor_data;
  portEXIT_CRITICAL(&sensor_mux);

  // Axis mapping (Matched to your frame configuration)
  float accX = -currentData.rawAccY; 
  float accY = -currentData.rawAccX; 
  float accZ = currentData.rawAccZ;
  
  float gRateX = (-currentData.rawGyY / 65.5f) - gyroErrorX;
  float gRateY = (-currentData.rawGyX / 65.5f) - gyroErrorY;
  float gRateZ = (currentData.rawGyZ / 65.5f) - gyroErrorZ;

  // Mahony filter requires gyro in Radians/Sec
  float gx = gRateX * 0.0174533f;
  float gy = gRateY * 0.0174533f;
  float gz = gRateZ * 0.0174533f;
  float ax = accX; 
  float ay = accY; 
  float az = accZ;

  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm; 
      ay *= recipNorm; 
      az *= recipNorm;

      // Estimated direction of gravity
      halfvx = q1 * q3 - q0 * q2;
      halfvy = q0 * q1 + q2 * q3;
      halfvz = q0 * q0 - 0.5f + q3 * q3;

      // Error is sum of cross product between estimated direction and measured direction of gravity
      halfex = (ay * halfvz - az * halfvy);
      halfey = (az * halfvx - ax * halfvz);
      halfez = (ax * halfvy - ay * halfvx);

      // Compute and apply integral feedback if enabled
      if(twoKiDef > 0.0f) {
          integralFBx += twoKiDef * halfex * dt;
          integralFBy += twoKiDef * halfey * dt;
          integralFBz += twoKiDef * halfez * dt;
          gx += integralFBx;
          gy += integralFBy;
          gz += integralFBz;
      } else {
          integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
      }

      // Apply proportional feedback
      gx += twoKpDef * halfex;
      gy += twoKpDef * halfey;
      gz += twoKpDef * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt); 
  gy *= (0.5f * dt); 
  gz *= (0.5f * dt);
  
  qa = q0; qb = q1; qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; 
  q1 *= recipNorm; 
  q2 *= recipNorm; 
  q3 *= recipNorm;

  // Convert to Euler angles (Degrees)
  actual_roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.2958f;
  actual_pitch = asin(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * 57.2958f;
  actual_yaw   = atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2958f;

  // Pack the expected struct for PID loop (Degrees/sec for gyro)
  return {actual_pitch, actual_roll, actual_yaw, gRateX, gRateY, gRateZ};
}
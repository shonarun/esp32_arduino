#include "control.h"
#include "config.h"
#include "sensors.h"
#include <Wire.h>

// Local PID tracking variables (encapsulated)
static float pitch_rate_integral = 0, roll_rate_integral = 0, yaw_rate_integral = 0;
static float prev_gyroRateX = 0, prev_gyroRateY = 0, prev_gyroRateZ = 0;

void initMotors() {
  ledcAttach(PIN_MOTOR_FL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_FR, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BR, MOT_FREQ, PWM_RESN);
  
  ledcWrite(PIN_MOTOR_FL, 0);
  ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0);
  ledcWrite(PIN_MOTOR_BR, 0);
}

void Task_FlightControl(void *pvParameters) {
  TickType_t xLastWakeTime;
  
  while(!calibrate_done) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, LOOP_TICKS);

    int16_t accX, accY, accZ, tempRaw, gyroX, gyroY, gyroZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (bytesReceived != 14) continue;

    int16_t rawAccX = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
    int16_t rawAccY = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
    int16_t rawAccZ = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
    tempRaw         = (Wire.read() << 8 | Wire.read());
    int16_t rawGyX  = (Wire.read() << 8 | Wire.read());
    int16_t rawGyY  = (Wire.read() << 8 | Wire.read());
    int16_t rawGyZ  = (Wire.read() << 8 | Wire.read());
    
    accX  = -rawAccY; accY  = -rawAccX; accZ  = rawAccZ;
    gyroX = -rawGyY;  gyroY = -rawGyX;  gyroZ = rawGyZ;

    float gyroRateX = (gyroX / 65.5) - gyroErrorX;
    float gyroRateY = (gyroY / 65.5) - gyroErrorY;
    float gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;

    static unsigned long prev_time = 0;
    unsigned long current_time = micros();
    float dt = (current_time - prev_time) / 1000000.0f;
    prev_time = current_time;

    Kalman_Update(gyroRateX, gyroRateY, gyroRateZ, (float)accX, (float)accY, (float)accZ, dt);

    if (emergency_kill || !is_flying) {
      pitch_rate_integral = 0; roll_rate_integral = 0; yaw_rate_integral = 0;
      ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
      ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
      continue; 
    }

    float local_target_pitch = target_pitch;
    float local_target_roll  = target_roll;
    float local_target_yaw_rate = target_yaw_rate;
    int base_t = current_base_thrust; 

    float pitch_angle_error = local_target_pitch - actual_pitch;
    float roll_angle_error  = local_target_roll  - actual_roll;

    float desired_pitch_rate = Kp_angle * pitch_angle_error; 
    float desired_roll_rate  = Kp_angle * roll_angle_error;

    float pitch_rate_error = desired_pitch_rate - gyroRateY;
    float roll_rate_error  = desired_roll_rate  - gyroRateX;
    float yaw_rate_error   = local_target_yaw_rate - gyroRateZ; 

    pitch_rate_integral += pitch_rate_error * dt;
    roll_rate_integral  += roll_rate_error  * dt;
    yaw_rate_integral   += yaw_rate_error   * dt;

    pitch_rate_integral = constrain(pitch_rate_integral, -400.0, 400.0);
    roll_rate_integral  = constrain(roll_rate_integral,  -400.0, 400.0);
    yaw_rate_integral   = constrain(yaw_rate_integral,   -400.0, 400.0);

    float pitch_pid_output = (Kp_rate * pitch_rate_error) + (Ki_rate * pitch_rate_integral) - (Kd_rate * (gyroRateY - prev_gyroRateY) / dt);
    float roll_pid_output  = (Kp_rate * roll_rate_error)  + (Ki_rate * roll_rate_integral)  - (Kd_rate * (gyroRateX - prev_gyroRateX) / dt);
    float yaw_pid_output   = (Kp_yaw_rate * yaw_rate_error) + (Ki_yaw_rate * yaw_rate_integral) - (Kd_yaw_rate * (gyroRateZ - prev_gyroRateZ) / dt);
    
    prev_gyroRateY = gyroRateY;
    prev_gyroRateX = gyroRateX;
    prev_gyroRateZ = gyroRateZ;

    int raw_FL = base_t + pitch_pid_output + roll_pid_output + yaw_pid_output;
    int raw_FR = base_t + pitch_pid_output - roll_pid_output - yaw_pid_output;
    int raw_BL = base_t - pitch_pid_output + roll_pid_output - yaw_pid_output;
    int raw_BR = base_t - pitch_pid_output - roll_pid_output + yaw_pid_output;

    ledcWrite(PIN_MOTOR_FL, constrain(raw_FL, 0, 800));
    ledcWrite(PIN_MOTOR_FR, constrain(raw_FR, 0, 800));
    ledcWrite(PIN_MOTOR_BL, constrain(raw_BL, 0, 800));
    ledcWrite(PIN_MOTOR_BR, constrain(raw_BR, 0, 800));
  }
}

void Task_Commander(void *pvParameters) {
  Serial.println("Drone booted. Waiting 8 seconds...");
  vTaskDelay(pdMS_TO_TICKS(8000)); 
  
  calibrateGyro();
  calibrate_done = true; 
  Serial.println("Gyro Calibrated...");

  unsigned long state_timer = 0; 

  for (;;) {
    if (emergency_kill && current_state != STATE_KILLED && current_state != STATE_BOOT) {
      current_state = STATE_KILLED;
      is_flying = false;
      is_armed = false;
      current_base_thrust = 0;
      Serial.println("FSM: EMERGENCY KILL TRIGGERED!");
    }

    switch (current_state) {
      case STATE_BOOT:
        Serial.println("STATUS: STANDBY. Waiting for ARM...");
        current_state = STATE_STANDBY;
        break;

      case STATE_STANDBY:
        if (is_armed && !emergency_kill) {
          Serial.println("STATUS: ARMED! Taking off in 3s...");
          state_timer = millis(); 
          current_state = STATE_ARMED_TAKEOFF;
        }
        break;

      case STATE_ARMED_TAKEOFF:
        if (millis() - state_timer >= 3000) {
          Serial.println("STATUS: FLYING!");
          state_timer = millis(); 
          target_pitch = 0.0;
          target_roll = 0.0;
          is_flying = true;
          current_state = STATE_FLYING;
        }
        break;

      case STATE_FLYING:
        {
          unsigned long time_in_air = millis() - state_timer;

          if (time_in_air < 1000) {
            current_base_thrust = map(time_in_air, 0, 1000, 0, HOVER_THRUST);
          } else if (time_in_air < 6000) {
            current_base_thrust = HOVER_THRUST;
          } else {
            Serial.println("STATUS: Flight Complete.");
            is_flying = false;
            current_base_thrust = 0;
            
            ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
            ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
            
            is_armed = false; 
            current_state = STATE_BOOT; 
          }
        }
        break;

      case STATE_KILLED:
        if (!emergency_kill) {
          Serial.println("STATUS: Kill switch cleared.");
          current_state = STATE_BOOT; 
        }
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}
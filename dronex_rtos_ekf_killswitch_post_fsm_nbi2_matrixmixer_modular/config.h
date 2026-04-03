#pragma once
#include <Arduino.h>

#define MOT_FREQ 20000
#define PWM_RESN 10

#define PIN_MOTOR_FL 4  
#define PIN_MOTOR_FR 5  
#define PIN_MOTOR_BL 3  
#define PIN_MOTOR_BR 6  

#define I2C_SDA 11      
#define I2C_SCL 10      

constexpr uint8_t MPU_ADDR = 0x68;
constexpr int16_t ACCEL_OFFSET_X = 1000;
constexpr int16_t ACCEL_OFFSET_Y = -700;
constexpr int16_t ACCEL_OFFSET_Z = 1160;

constexpr int HOVER_THRUST = 511; 
constexpr float LOOP_TIME_SEC = 0.004; // 250Hz
#define LOOP_TICKS pdMS_TO_TICKS(4)

extern volatile int current_base_thrust;
extern volatile float target_pitch;
extern volatile float target_roll;
extern volatile float target_yaw_rate;

extern float Kp_angle; 
extern float Kp_rate, Ki_rate, Kd_rate;
extern float Kp_yaw_rate, Ki_yaw_rate, Kd_yaw_rate;

extern TaskHandle_t FlightControlTask;
extern TaskHandle_t CommanderTask;
extern TaskHandle_t CommsTask;
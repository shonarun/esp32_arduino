#pragma once
#include <Arduino.h>

// ==========================================
// HARDWARE PINS & CONFIG
// ==========================================
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

// Flight Settings
constexpr int HOVER_THRUST = 511; 
constexpr float LOOP_TIME_SEC = 0.004; // 250Hz
#define LOOP_TICKS pdMS_TO_TICKS(4)

// ==========================================
// SHARED TYPES & GLOBALS
// ==========================================
enum FlightState {
  STATE_BOOT,
  STATE_STANDBY,
  STATE_ARMED_TAKEOFF,
  STATE_FLYING,
  STATE_KILLED
};

// State flags
extern volatile FlightState current_state;
extern volatile bool emergency_kill;
extern volatile bool is_armed;
extern volatile bool is_flying;
extern volatile bool calibrate_done;
extern volatile int current_base_thrust;

// Targets & Actuals
extern volatile float target_pitch;
extern volatile float target_roll;
extern volatile float target_yaw_rate;
extern float actual_pitch;
extern float actual_roll;
extern float actual_yaw;

// Gyro Errors (needed for reading in control loop)
extern float gyroErrorX;
extern float gyroErrorY;
extern float gyroErrorZ;

// PID Tuning
extern float Kp_angle; 
extern float Kp_rate, Ki_rate, Kd_rate;
extern float Kp_yaw_rate, Ki_yaw_rate, Kd_yaw_rate;

// RTOS Handles
extern TaskHandle_t FlightControlTask;
extern TaskHandle_t CommanderTask;
extern TaskHandle_t CommsTask;
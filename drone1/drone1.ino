#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#define MOT_FREQ 20000
#define PWM_RESN 10

Adafruit_MPU6050 mpu;

// ==========================================
// LITEWING v1.2 HARDWARE PINS (ESP32-S3)
// ==========================================
const int PIN_MOTOR_FL = 4;  // Front Left
const int PIN_MOTOR_FR = 5;  // Front Right
const int PIN_MOTOR_BL = 3;  // Back Left
const int PIN_MOTOR_BR = 6;  // Back Right

const int I2C_SDA = 11;      // MPU6050 SDA
const int I2C_SCL = 10;      // MPU6050 SCL

// ==========================================
// FLIGHT SETTINGS
// ==========================================
const int HOVER_THRUST = 511; 

// PID Tuning values
float Kp = 3.0, Ki = 0.0, Kd = 0.0;

float pitch_integral = 0, pitch_prev_error = 0;
float roll_integral = 0, roll_prev_error = 0;

unsigned long flight_start_time = 0;
unsigned long previous_time = 0;
bool is_flying = false;

// Filtered Angles
float actual_pitch = 0.0;
float actual_roll = 0.0;

void setup() {
  Serial.begin(115200);
  
  // FIXED: Using correct pin variables for ESP32 Core 3.x ledc API
  ledcAttach(PIN_MOTOR_FL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_FR, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BR, MOT_FREQ, PWM_RESN);

  // Initialize the MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) { delay(10); } 
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Drone booted. Waiting 10 seconds...");
  delay(10000); 
  
  Serial.println("Starting Flight Sequence!");
  flight_start_time = millis();
  previous_time = micros(); // Track microseconds for precise dt
  is_flying = true;
}

void loop() {
  if (!is_flying) return;

  unsigned long current_time_ms = millis();
  unsigned long current_time_us = micros();
  
  // Calculate dt in seconds for PID and Filters
  float dt = (current_time_us - previous_time) / 1000000.0;
  previous_time = current_time_us;
  
  unsigned long time_in_air = current_time_ms - flight_start_time;
  int current_base_thrust = 0;

  // Read IMU Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // FIXED: Corrected Axes for Standard X-Forward orientation
  float accel_pitch = atan2(-a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  float accel_roll = atan2(a.acceleration.x,sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // FIXED: Implemented a Complementary Filter (combines Gyro and Accel)
  // Gyro provides fast response, Accel prevents long-term drift
  actual_pitch = 0.98 * (actual_pitch + (g.gyro.x * dt * 180 / PI)) + 0.02 * accel_pitch;
  actual_roll = 0.98 * (actual_roll + (g.gyro.y * dt * 180 / PI)) + 0.02 * accel_roll;

  // Flight Timeline
  if (time_in_air < 1000) {
    current_base_thrust = map(time_in_air, 0, 1000, 0, HOVER_THRUST);
  } else if (time_in_air < 6000) {
    current_base_thrust = HOVER_THRUST;
  } else {
    // Land
    ledcWrite(PIN_MOTOR_FL, 0);
    ledcWrite(PIN_MOTOR_FR, 0);
    ledcWrite(PIN_MOTOR_BL, 0);
    ledcWrite(PIN_MOTOR_BR, 0);
    Serial.println("Flight Complete.");
    is_flying = false;
    return;
  }

  // Run stabilization
  stabilize_attitude(current_base_thrust, dt);
}

void stabilize_attitude(int base_thrust, float dt) {
  // Prevent divide-by-zero on first loop
  if (dt <= 0.0) return; 

  float pitch_error = 1.74 - actual_pitch;
  float roll_error = 2.50 - actual_roll;

  // FIXED: Proper discrete PID math incorporating dt
  pitch_integral += (pitch_error * dt);
  float pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  float pitch_pid = (Kp * pitch_error) + (Ki * pitch_integral) + (Kd * pitch_derivative);
  pitch_prev_error = pitch_error;

  roll_integral += (roll_error * dt);
  float roll_derivative = (roll_error - roll_prev_error) / dt;
  float roll_pid = (Kp * roll_error) + (Ki * roll_integral) + (Kd * roll_derivative);
  roll_prev_error = roll_error;

  // Motor Mixing
  int speed_FL = base_thrust + pitch_pid + roll_pid;
  int speed_FR = base_thrust + pitch_pid - roll_pid;
  int speed_BL = base_thrust - pitch_pid + roll_pid;
  int speed_BR = base_thrust - pitch_pid - roll_pid;

  speed_FL = constrain(speed_FL, 0, 800);
  speed_FR = constrain(speed_FR, 0, 800);
  speed_BL = constrain(speed_BL, 0, 800);
  speed_BR = constrain(speed_BR, 0, 800);

  // Write speeds
  ledcWrite(PIN_MOTOR_FL, speed_FL);
  ledcWrite(PIN_MOTOR_FR, speed_FR);
  ledcWrite(PIN_MOTOR_BL, speed_BL);
  ledcWrite(PIN_MOTOR_BR, speed_BR);
}
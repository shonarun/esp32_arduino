#include <Wire.h>

// --- Hardware Definitions ---
#define IMU_SDA 11
#define IMU_SCL 10

#define MOT_1 5 // Front Right (CCW)
#define MOT_2 6 // Rear Right (CW)
#define MOT_3 3 // Rear Left (CCW)
#define MOT_4 4 // Front Left (CW)

// --- PID Configuration ---
// Struct to hold PID data for clean organization
struct PID {
  float P, I, D;
  float error, prev_error;
  float integral, derivative;
  float output;
};

// Tune these values carefully (Kp, Ki, Kd)
float pitch_Kp = 1.2, pitch_Ki = 0.04, pitch_Kd = 15.0;
float roll_Kp  = 1.2, roll_Ki  = 0.04, roll_Kd  = 15.0;
float yaw_Kp   = 2.0, yaw_Ki   = 0.02, yaw_Kd   = 0.0;

PID pitchPID = {pitch_Kp, pitch_Ki, pitch_Kd, 0, 0, 0, 0, 0};
PID rollPID  = {roll_Kp, roll_Ki, roll_Kd, 0, 0, 0, 0, 0};
PID yawPID   = {yaw_Kp, yaw_Ki, yaw_Kd, 0, 0, 0, 0, 0};

// --- Flight Variables ---
float desired_pitch = 0, desired_roll = 0, desired_yaw = 0;
float current_pitch = 0, current_roll = 0, current_yaw = 0; // Populated by MPU6000
int base_throttle = 1300; // Base hover throttle (1000 - 2000 range)

// Loop timing
unsigned long prev_time;
float dt;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for MPU6000
  Wire.begin(IMU_SDA, IMU_SCL);
  initMPU6000();

  // Configure ESP32 Motor PWM using standard LEDC timer
  ledcSetup(0, 400, 10); // Channel 0, 400Hz for ESCs, 10-bit resolution
  ledcSetup(1, 400, 10);
  ledcSetup(2, 400, 10);
  ledcSetup(3, 400, 10);

  ledcAttachPin(MOT_1, 0);
  ledcAttachPin(MOT_2, 1);
  ledcAttachPin(MOT_3, 2);
  ledcAttachPin(MOT_4, 3);
  
  prev_time = micros();
}

void loop() {
  // 1. Calculate delta time (dt) in seconds
  unsigned long current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
  prev_time = current_time;

  // 2. Read IMU Data (Pseudo-function, requires actual register reading math)
  readIMU(&current_pitch, &current_roll, &current_yaw); 

  // 3. Compute PID Outputs
  calculatePID(&pitchPID, desired_pitch, current_pitch, dt);
  calculatePID(&rollPID, desired_roll, current_roll, dt);
  calculatePID(&yawPID, desired_yaw, current_yaw, dt);

  // 4. Motor Mixing (Standard Quad X Configuration)
  // Utilizing basic linear vector additions to mix the axes
  int m1_pwm = base_throttle - pitchPID.output + rollPID.output - yawPID.output; 
  int m2_pwm = base_throttle + pitchPID.output + rollPID.output + yawPID.output; 
  int m3_pwm = base_throttle + pitchPID.output - rollPID.output - yawPID.output; 
  int m4_pwm = base_throttle - pitchPID.output - rollPID.output + yawPID.output; 

  // Constrain outputs to prevent ESC sync loss
  m1_pwm = constrain(m1_pwm, 1000, 2000);
  m2_pwm = constrain(m2_pwm, 1000, 2000);
  m3_pwm = constrain(m3_pwm, 1000, 2000);
  m4_pwm = constrain(m4_pwm, 1000, 2000);

  // 5. Write to Motors
  writeMotors(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
}

void calculatePID(PID *pid, float setpoint, float measured, float dt) {
  pid->error = setpoint - measured;
  
  // Proportional
  float P_out = pid->P * pid->error;
  
  // Integral (with basic anti-windup)
  pid->integral += pid->error * dt;
  pid->integral = constrain(pid->integral, -400, 400); 
  float I_out = pid->I * pid->integral;
  
  // Derivative
  pid->derivative = (pid->error - pid->prev_error) / dt;
  float D_out = pid->D * pid->derivative;
  
  pid->output = P_out + I_out + D_out;
  pid->prev_error = pid->error;
}

void writeMotors(int m1, int m2, int m3, int m4) {
  // Map 1000-2000ms standard RC signal to 10-bit PWM (0-1023)
  ledcWrite(0, map(m1, 0, 2500, 0, 1023)); 
  ledcWrite(1, map(m2, 0, 2500, 0, 1023));
  ledcWrite(2, map(m3, 0, 2500, 0, 1023));
  ledcWrite(3, map(m4, 0, 2500, 0, 1023));
}

void initMPU6000() {
  // Basic Wake-up for MPU6000
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-6000)
  Wire.endTransmission(true);
}

void readIMU(float *pitch, float *roll, float *yaw) {
  // Add complementary or Kalman filter logic here to fuse Gyro and Accelerometer data
}
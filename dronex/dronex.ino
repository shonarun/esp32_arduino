#include <Wire.h>

#define MOT_FREQ 20000
#define PWM_RESN 10

// ==========================================
// LITEWING v1.2 HARDWARE PINS (ESP32-S3)
// ==========================================
const int PIN_MOTOR_FL = 4;  // Front Left
const int PIN_MOTOR_FR = 5;  // Front Right
const int PIN_MOTOR_BL = 3;  // Back Left
const int PIN_MOTOR_BR = 6;  // Back Right

#define I2C_SDA 11      // MPU6050 SDA
#define I2C_SCL 10      // MPU6050 SCL

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

// ==========================================
// FLIGHT SETTINGS
// ==========================================
const int HOVER_THRUST = 511; 

// PID Tuning values
float Kp = 2.0, Ki = 0.0, Kd = 0.0;

float pitch_integral = 0, pitch_prev_error = 0;
float roll_integral = 0, roll_prev_error = 0;

float gyroRateX = 0.0;
float gyroRateY = 0.0;
float gyroRateZ = 0.0;

unsigned long flight_start_time = 0;
unsigned long previous_time = 0;
bool is_flying = false;

// Filtered Angles
float actual_pitch = 0.0;
float actual_roll  = 0.0;

void setup() {
  Serial.begin(115200);
  
  // FIXED: Using correct pin variables for ESP32 Core 3.x ledc API
  ledcAttach(PIN_MOTOR_FL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_FR, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BR, MOT_FREQ, PWM_RESN);
  
  ledcWrite(PIN_MOTOR_FL, 0);
  ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0);
  ledcWrite(PIN_MOTOR_BR, 0);

  // Initialize the MPU6050
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

  Serial.println("Drone booted. Waiting 10 seconds...");
  calibrateGyro();
  delay(8000); 
  
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
  
  gyroRateX = (gyroX / 65.5) - gyroErrorX;
  gyroRateY = (gyroY / 65.5) - gyroErrorY;
  gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;

  // FIXED: Corrected Axes for Standard X-Forward orientation
  float accel_pitch = atan2(-accY, sqrt((float)accX * accX + (float)accZ * accZ)) * 57.2958;
  float accel_roll  = atan2( accX, sqrt((float)accY * accY + (float)accZ * accZ)) * 57.2958;

  // FIXED: Implemented a Complementary Filter (combines Gyro and Accel)
  // Gyro provides fast response, Accel prevents long-term drift
  actual_pitch = 0.98 * (actual_pitch + (gyroRateX * dt)) + 0.02 * accel_pitch;
  actual_roll  = 0.98 * (actual_roll  + (gyroRateY * dt)) + 0.02 * accel_roll;

  // Flight Timeline
  if (time_in_air < 1000) {
    current_base_thrust = map(time_in_air, 0, 1000, 0, HOVER_THRUST);
  } else if (time_in_air < 5000) {
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

  float pitch_error = 0.0 - actual_pitch;
  float roll_error  = 0.0 - actual_roll;

  // FIXED: Proper discrete PID math incorporating dt
  pitch_integral += (pitch_error * dt);
  // float pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  float pitch_pid = (Kp * pitch_error) + (Ki * pitch_integral) - (Kd * gyroRateX);
  pitch_prev_error = pitch_error;

  roll_integral += (roll_error * dt);
  // float roll_derivative = (roll_error - roll_prev_error) / dt;
  float roll_pid = (Kp * roll_error) + (Ki * roll_integral) - (Kd * gyroRateY);
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
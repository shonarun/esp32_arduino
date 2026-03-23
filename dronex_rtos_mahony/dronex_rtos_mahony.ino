#include <Wire.h>
#include <Arduino.h>

#define MOT_FREQ 20000
#define PWM_RESN 10

#define MPU_FILTER_CONFIG 0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C

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

// ==========================================
// RTOS TASK HANDLES & SHARED VARIABLES
// ==========================================
TaskHandle_t FlightControlTask;
TaskHandle_t CommanderTask;

// Volatile because they are modified by Core 0 and read by Core 1
volatile int current_base_thrust = 0;
volatile bool is_flying = false;
volatile bool calibrate_done = false;

// ROS/Commander Targets
volatile float target_pitch = 0.0;
volatile float target_roll  = 0.0;

// ==========================================
// FLIGHT SETTINGS & CASCADED PID
// ==========================================
const int HOVER_THRUST = 511; 
const float LOOP_TIME_SEC = 0.004; // 4ms loop = 250Hz
const TickType_t LOOP_TICKS = pdMS_TO_TICKS(4); 

// Outer Loop (Angle)
float Kp_angle = 2.0; 

// Inner Loop (Rate)
float Kp_rate = 1.0, Ki_rate = 0.0, Kd_rate = 0.001;
float pitch_rate_integral = 0, roll_rate_integral = 0;
float pitch_prev_rate_error = 0, roll_prev_rate_error = 0;

float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;
float actual_pitch = 0.0, actual_roll  = 0.0;

// ==========================================
// MAHONY FILTER GLOBALS
// ==========================================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
float twoKpDef = 1.0f; 
float twoKiDef = 0.0f; 
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

// Function Prototypes
void calibrateGyro();
void Task_FlightControl(void *pvParameters);
void Task_Commander(void *pvParameters);
float invSqrt(float x);
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

void setup() {
  Serial.begin(115200);
  
  // Initialize Motors
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

  // Create the FreeRTOS Tasks
  // Core 1: High Priority Flight Controller
  xTaskCreatePinnedToCore(
    Task_FlightControl,   
    "FlightControl",      
    4096,                 
    NULL,                 
    2,                    
    &FlightControlTask,   
    1                     
  );

  // Core 0: Lower Priority Commander
  xTaskCreatePinnedToCore(
    Task_Commander,
    "Commander",
    4096,
    NULL,
    1,
    &CommanderTask,
    0
  );
}

void loop() {
  vTaskDelete(NULL); 
}

// ==========================================
// TASK 1: STRICT TIMING CONTROL LOOP (Core 1)
// ==========================================
void Task_FlightControl(void *pvParameters) {
  TickType_t xLastWakeTime;
  
  while(!calibrate_done) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, LOOP_TICKS);

    int16_t accX, accY, accZ, tempRaw, gyroX, gyroY, gyroZ;

    // Read IMU Data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
    
    accX    = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
    accY    = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
    accZ    = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
    tempRaw = (Wire.read() << 8 | Wire.read());
    gyroX   = (Wire.read() << 8 | Wire.read());
    gyroY   = (Wire.read() << 8 | Wire.read());
    gyroZ   = (Wire.read() << 8 | Wire.read());

    // Gyro Rates (Degrees/sec for PID)
    float gyroRateX = (gyroX / 65.5) - gyroErrorX;
    float gyroRateY = (gyroY / 65.5) - gyroErrorY;
    float gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;

    // Gyro Rates (Radians/sec for Mahony)
    float gyroRadX = gyroRateX * 0.0174533f;
    float gyroRadY = gyroRateY * 0.0174533f;
    float gyroRadZ = gyroRateZ * 0.0174533f;

    // Sensor Fusion Update
    Mahony_Update(gyroRadX, gyroRadY, gyroRadZ, (float)accX, (float)accY, (float)accZ, LOOP_TIME_SEC);

    if (!is_flying) {
      pitch_rate_integral = 0;
      roll_rate_integral = 0;
      continue;
    }

    // Grab thread-safe local copies of commander targets
    float local_target_pitch = target_pitch;
    float local_target_roll  = target_roll;
    int base_t = current_base_thrust; 

    // OUTER LOOP: Angle PID
    float pitch_angle_error = local_target_pitch - actual_pitch;
    float roll_angle_error  = local_target_roll  - actual_roll;

    float desired_pitch_rate = Kp_angle * pitch_angle_error; 
    float desired_roll_rate  = Kp_angle * roll_angle_error;

    // INNER LOOP: Rate PID 
    float pitch_rate_error = desired_pitch_rate - gyroRateX;
    float roll_rate_error  = desired_roll_rate  - gyroRateY;

    pitch_rate_integral += pitch_rate_error * LOOP_TIME_SEC;
    roll_rate_integral  += roll_rate_error  * LOOP_TIME_SEC;

    float pitch_pid_output = (Kp_rate * pitch_rate_error) + (Ki_rate * pitch_rate_integral) + (Kd_rate * (pitch_rate_error - pitch_prev_rate_error) / LOOP_TIME_SEC);
    float roll_pid_output  = (Kp_rate * roll_rate_error)  + (Ki_rate * roll_rate_integral)  + (Kd_rate * (roll_rate_error - roll_prev_rate_error) / LOOP_TIME_SEC);

    pitch_prev_rate_error = pitch_rate_error;
    roll_prev_rate_error  = roll_rate_error;

    // Motor Mixing
    int speed_FL = constrain(base_t + pitch_pid_output + roll_pid_output, 0, 800);
    int speed_FR = constrain(base_t + pitch_pid_output - roll_pid_output, 0, 800);
    int speed_BL = constrain(base_t - pitch_pid_output + roll_pid_output, 0, 800);
    int speed_BR = constrain(base_t - pitch_pid_output - roll_pid_output, 0, 800);

    // Write speeds
    ledcWrite(PIN_MOTOR_FL, speed_FL);
    ledcWrite(PIN_MOTOR_FR, speed_FR);
    ledcWrite(PIN_MOTOR_BL, speed_BL);
    ledcWrite(PIN_MOTOR_BR, speed_BR);
  }
}

// ==========================================
// TASK 2: FLIGHT COMMANDER (Core 0)
// ==========================================
void Task_Commander(void *pvParameters) {
  
  Serial.println("Drone booted. Waiting 8 seconds...");
  calibrateGyro();
  calibrate_done = true; 
  Serial.println("Gyro Calibrated...");

  vTaskDelay(pdMS_TO_TICKS(8000)); 
  
  Serial.println("Starting Flight Sequence!");
  unsigned long flight_start_time = millis();
  
  target_pitch = 0.0;
  target_roll = 0.0;
  is_flying = true;

  for (;;) {
    if (!is_flying) {
      vTaskDelay(pdMS_TO_TICKS(100)); 
      continue;
    }

    unsigned long time_in_air = millis() - flight_start_time;

    if (time_in_air < 1000) {
      current_base_thrust = map(time_in_air, 0, 1000, 0, HOVER_THRUST);
    } 
    else if (time_in_air < 5000) {
      current_base_thrust = HOVER_THRUST;
    } 
    else {
      is_flying = false;
      current_base_thrust = 0;
      
      ledcWrite(PIN_MOTOR_FL, 0);
      ledcWrite(PIN_MOTOR_FR, 0);
      ledcWrite(PIN_MOTOR_BL, 0);
      ledcWrite(PIN_MOTOR_BR, 0);
      
      Serial.println("Flight Complete.");
    }

    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

// ==========================================
// HELPER FUNCTIONS & SENSOR FUSION
// ==========================================
void calibrateGyro() {
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
  
  gyroErrorX /= 2000;
  gyroErrorY /= 2000;
  gyroErrorZ /= 2000;
}

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

    integralFBx += twoKiDef * halfex * dt;
    integralFBy += twoKiDef * halfey * dt;
    integralFBz += twoKiDef * halfez * dt;

    gx += twoKpDef * halfex + integralFBx;
    gy += twoKpDef * halfey + integralFBy;
    gz += twoKpDef * halfez + integralFBz;

    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    actual_roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.2958f;
    actual_pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * 57.2958f;
}
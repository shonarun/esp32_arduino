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

// ==========================================
// FLIGHT SETTINGS
// ==========================================
const int HOVER_THRUST = 511; 
const float LOOP_TIME_SEC = 0.004; // 4ms loop = 250Hz
const TickType_t LOOP_TICKS = pdMS_TO_TICKS(4); 

// PID Tuning values
float Kp = 1.0, Ki = 0.0, Kd = 0.001;
float pitch_integral = 0, pitch_prev_error = 0;
float roll_integral = 0, roll_prev_error = 0;

float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;
float actual_pitch = 0.0, actual_roll  = 0.0;

// Function Prototypes
void calibrateGyro();
void Task_FlightControl(void *pvParameters);
void Task_Commander(void *pvParameters);

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
    Task_FlightControl,   // Function to implement the task
    "FlightControl",      // Name of the task
    4096,                 // Stack size in words
    NULL,                 // Task input parameter
    2,                    // Priority of the task (Higher = more priority)
    &FlightControlTask,   // Task handle
    1                     // Core where the task should run
  );

  // Core 0: Lower Priority Commander (Timeline & Serial)
  xTaskCreatePinnedToCore(
    Task_Commander,
    "Commander",
    4096,
    NULL,
    1,
    &CommanderTask,
    0
  );

  // Setup is complete. The Arduino loop() is no longer needed.
}

void loop() {
  // Empty. FreeRTOS handles everything now.
  vTaskDelete(NULL); 
}

// ==========================================
// TASK 1: STRICT TIMING CONTROL LOOP (Core 1)
// ==========================================
void Task_FlightControl(void *pvParameters) {
  TickType_t xLastWakeTime;
  
  // Wait for calibration to finish before starting the loop
  while(!calibrate_done) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Initialize the WakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // This function ensures the loop runs EXACTLY every LOOP_TICKS (4ms)
    // No need to calculate 'dt' anymore, it is perfectly constant.
    vTaskDelayUntil(&xLastWakeTime, LOOP_TICKS);

    if (!is_flying) {
      // Keep integrals at 0 when not flying to prevent I-term windup
      pitch_integral = 0;
      roll_integral = 0;
      continue;
    }

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

    float gyroRateX = (gyroX / 65.5) - gyroErrorX;
    float gyroRateY = (gyroY / 65.5) - gyroErrorY;
    float gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;

    float accel_pitch = atan2(-accY, sqrt((float)accX * accX + (float)accZ * accZ)) * 57.2958;
    float accel_roll  = atan2( accX, sqrt((float)accY * accY + (float)accZ * accZ)) * 57.2958;

    actual_pitch = 0.98f * (actual_pitch + (gyroRateX * LOOP_TIME_SEC)) + 0.02f * accel_pitch;
    actual_roll  = 0.98f * (actual_roll  + (gyroRateY * LOOP_TIME_SEC)) + 0.02f * accel_roll;

    // PID Math using constant LOOP_TIME_SEC
    float pitch_error = 0.0 - actual_pitch;
    float roll_error  = 0.0 - actual_roll;

    pitch_integral += (pitch_error * LOOP_TIME_SEC);
    float pitch_pid = (Kp * pitch_error) + (Ki * pitch_integral) - (Kd * gyroRateX);

    roll_integral += (roll_error * LOOP_TIME_SEC);
    float roll_pid = (Kp * roll_error) + (Ki * roll_integral) - (Kd * gyroRateY);

    // Grab a local copy of the volatile variable
    int base_t = current_base_thrust; 

    // Motor Mixing
    int speed_FL = constrain(base_t + pitch_pid + roll_pid, 0, 800);
    int speed_FR = constrain(base_t + pitch_pid - roll_pid, 0, 800);
    int speed_BL = constrain(base_t - pitch_pid + roll_pid, 0, 800);
    int speed_BR = constrain(base_t - pitch_pid - roll_pid, 0, 800);

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
  
  Serial.println("Drone booted. Waiting 10 seconds...");
  calibrateGyro();
  calibrate_done = true; // Signals Core 1 it can start running the control loop
  Serial.println("Gyro Calibrated...");

  vTaskDelay(pdMS_TO_TICKS(8000)); // Non-blocking delay
  
  Serial.println("Starting Flight Sequence!");
  unsigned long flight_start_time = millis();
  is_flying = true;

  for (;;) {
    if (!is_flying) {
      vTaskDelay(pdMS_TO_TICKS(100)); // Just idle
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
      // Land
      is_flying = false;
      current_base_thrust = 0;
      
      ledcWrite(PIN_MOTOR_FL, 0);
      ledcWrite(PIN_MOTOR_FR, 0);
      ledcWrite(PIN_MOTOR_BL, 0);
      ledcWrite(PIN_MOTOR_BR, 0);
      
      Serial.println("Flight Complete.");
    }

    // Run this state machine at 50Hz (20ms)
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

// ==========================================
// HELPER FUNCTIONS
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
    
    vTaskDelay(pdMS_TO_TICKS(3)); // RTOS friendly delay!
  }
  
  gyroErrorX /= 2000;
  gyroErrorY /= 2000;
  gyroErrorZ /= 2000;
}
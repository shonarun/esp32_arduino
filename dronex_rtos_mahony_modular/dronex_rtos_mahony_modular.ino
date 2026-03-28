#include <Arduino.h>
#include "sensors.h"
#include "estimator.h"
#include "pid.h"
#include "mixer.h"

// RTOS Task Handles & Shared Variables
TaskHandle_t FlightControlTask;
TaskHandle_t CommanderTask;

volatile int current_base_thrust = 0;
volatile bool is_flying = false;
volatile bool calibrate_done = false;

volatile float target_pitch = 0.0;
volatile float target_roll  = 0.0;
volatile float target_yaw_rate = 0.0; 

const int HOVER_THRUST = 511; 
const float LOOP_TIME_SEC = 0.004; // 250Hz
const TickType_t LOOP_TICKS = pdMS_TO_TICKS(4); 

// Function Prototypes
void Task_FlightControl(void *pvParameters);
void Task_Commander(void *pvParameters);

void setup() {
  Serial.begin(115200);
  
  Mixer_Init();
  Sensors_Init();

  xTaskCreatePinnedToCore(Task_FlightControl, "FlightControl", 4096, NULL, 2, &FlightControlTask, 1);
  xTaskCreatePinnedToCore(Task_Commander, "Commander", 4096, NULL, 1, &CommanderTask, 0);
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

    float gyroRadX, gyroRadY, gyroRadZ;
    float accX, accY, accZ;
    float gyroRateX, gyroRateY, gyroRateZ;

    // 1. Read Sensors
    Sensors_Read(gyroRadX, gyroRadY, gyroRadZ, accX, accY, accZ, gyroRateX, gyroRateY, gyroRateZ);

    // 2. Estimate Attitude
    Estimator_Update(gyroRadX, gyroRadY, gyroRadZ, accX, accY, accZ, LOOP_TIME_SEC);

    if (!is_flying) {
      PID_Reset();
      Mixer_Stop();
      continue;
    }

    // Grab thread-safe local copies
    float local_target_pitch = target_pitch;
    float local_target_roll  = target_roll;
    float local_target_yaw_rate = target_yaw_rate;
    int base_t = current_base_thrust; 
    
    float actual_pitch = Estimator_GetPitch();
    float actual_roll = Estimator_GetRoll();

    // 3. Compute PID
    float pitch_pid, roll_pid, yaw_pid;
    PID_Compute(local_target_pitch, local_target_roll, local_target_yaw_rate,
                actual_pitch, actual_roll,
                gyroRateX, gyroRateY, gyroRateZ,
                LOOP_TIME_SEC, 
                pitch_pid, roll_pid, yaw_pid);

    // 4. Mix and Output to Motors
    Mixer_Write(base_t, pitch_pid, roll_pid, yaw_pid);
  }
}

// ==========================================
// TASK 2: FLIGHT COMMANDER (Core 0)
// ==========================================
void Task_Commander(void *pvParameters) {
  Serial.println("Drone booted. Waiting 8 seconds...");
  
  Sensors_Calibrate();
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
    else if (time_in_air < 6000) {
      current_base_thrust = HOVER_THRUST;
    } 
    else {
      is_flying = false;
      current_base_thrust = 0;
      Mixer_Stop();
      Serial.println("Flight Complete.");
    }

    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}
#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "estimator.h"
#include "pid.h"
#include "mixer.h"
#include "comms.h"

void Task_FlightControl(void *pvParameters);
void Task_Commander(void *pvParameters);

void setup() {
  Serial.begin(115200);
  initMixer();
  initEstimator();

  xTaskCreatePinnedToCore(Task_Comms, "CommsTask", 10240, NULL, 2, &CommsTask, 0);
  xTaskCreatePinnedToCore(Task_FlightControl, "FlightControl", 4096, NULL, 2, &FlightControlTask, 1);
  xTaskCreatePinnedToCore(Task_Commander, "Commander", 4096, NULL, 1, &CommanderTask, 0);
}

void loop() { vTaskDelete(NULL); }

// ====================================================
// CORE 1: STRICT TIMING CONTROL LOOP
// ====================================================
void Task_FlightControl(void *pvParameters) {
  TickType_t xLastWakeTime;
  while(!calibrate_done) { vTaskDelay(pdMS_TO_TICKS(100)); }
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, LOOP_TICKS);

    static unsigned long prev_time = 0;
    unsigned long current_time = micros();
    float dt = (current_time - prev_time) / 1000000.0f;
    prev_time = current_time;

    AttitudeData attitude = updateEstimator(dt);

    if (emergency_kill || !is_flying) {
      resetPID();
      killMotors();
      continue; 
    }

    PID_Output corrections = calculatePID(attitude, dt);
    applyMotorMixing(current_base_thrust, corrections);
  }
}

// ====================================================
// CORE 0: STATE MACHINE COMMANDER
// ====================================================
void Task_Commander(void *pvParameters) {
  setFlightState(STATE_BOOT);
  vTaskDelay(pdMS_TO_TICKS(8000)); 
  
  calibrateEstimator();
  calibrate_done = true; 

  unsigned long state_timer = 0; 

  for (;;) {
    if (emergency_kill && current_state != STATE_KILLED && current_state != STATE_BOOT) {
      setFlightState(STATE_KILLED);
      is_flying = false; is_armed = false; current_base_thrust = 0;
    }

    switch (current_state) {
      case STATE_BOOT:
        setFlightState(STATE_STANDBY);
        break;

      case STATE_STANDBY:
        if (is_armed && !emergency_kill) {
          state_timer = millis(); 
          setFlightState(STATE_ARMED_TAKEOFF);
        }
        break;

      case STATE_ARMED_TAKEOFF:
        if (millis() - state_timer >= 3000) {
          state_timer = millis(); 
          target_pitch = 0.0; target_roll = 0.0;
          is_flying = true; setFlightState(STATE_FLYING);
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
            is_flying = false; current_base_thrust = 0; killMotors();
            is_armed = false; setFlightState(STATE_BOOT); 
          }
        }
        break;

      case STATE_KILLED:
        if (!emergency_kill) setFlightState(STATE_BOOT); 
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}
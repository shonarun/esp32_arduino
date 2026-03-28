#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "control.h"
#include "comms.h"

void setup() {
  Serial.begin(115200);
  
  initMotors();
  initMPU();

  // Spin up RTOS Tasks
  xTaskCreatePinnedToCore(Task_Comms, "CommsTask", 10240, NULL, 2, &CommsTask, 0);
  xTaskCreatePinnedToCore(Task_FlightControl, "FlightControl", 4096, NULL, 2, &FlightControlTask, 1);
  xTaskCreatePinnedToCore(Task_Commander, "Commander", 4096, NULL, 1, &CommanderTask, 0);
}

void loop() {
  // Main loop is not used; everything is handled by FreeRTOS.
  vTaskDelete(NULL); 
}
#pragma once
#include <Arduino.h>

void initMotors();
void Task_FlightControl(void *pvParameters);
void Task_Commander(void *pvParameters);
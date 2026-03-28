#pragma once
#include <Arduino.h>

void initMPU();
void calibrateGyro();
void Kalman_Update(float gyroRateX, float gyroRateY, float gyroRateZ, float accX, float accY, float accZ, float dt);
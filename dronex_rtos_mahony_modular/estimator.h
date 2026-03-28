#pragma once
#include <Arduino.h>

void Estimator_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
float Estimator_GetPitch();
float Estimator_GetRoll();
float Estimator_GetYaw();
#pragma once
#include <Arduino.h>

void PID_Reset();
void PID_Compute(float target_pitch, float target_roll, float target_yaw_rate,
                 float actual_pitch, float actual_roll,
                 float gyroRateX, float gyroRateY, float gyroRateZ,
                 float dt, 
                 float &pitch_out, float &roll_out, float &yaw_out);
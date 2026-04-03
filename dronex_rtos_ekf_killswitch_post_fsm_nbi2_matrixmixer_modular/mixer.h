#pragma once
#include "pid.h"

void initMixer();
void applyMotorMixing(int base_thrust, PID_Output pid);
void killMotors();

constexpr float MOTOR_MAX_PWM = 1023.0f;
constexpr float MOTOR_MIN_IDLE = 40.0f; 

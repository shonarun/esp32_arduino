#pragma once
#include "pid.h"

void initMixer();
void applyMotorMixing(int base_thrust, PID_Output pid);
void killMotors();

constexpr float MOTOR_MAX_PWM = 1023.0f; // 10-bit max
constexpr float MOTOR_MIN_IDLE = 40.0f;  // Minimum to keep props spinning smoothly

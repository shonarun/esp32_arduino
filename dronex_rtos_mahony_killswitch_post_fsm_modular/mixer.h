#pragma once
#include "pid.h"

void initMixer();
void applyMotorMixing(int base_thrust, PID_Output pid);
void killMotors();
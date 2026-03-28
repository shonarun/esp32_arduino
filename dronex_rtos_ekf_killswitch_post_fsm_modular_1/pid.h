#pragma once
#include "estimator.h"

struct PID_Output { float pitch; float roll; float yaw; };

void resetPID();
PID_Output calculatePID(AttitudeData current_attitude, float dt);
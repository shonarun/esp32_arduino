#pragma once

struct AttitudeData {
    float pitch;
    float roll;
    float yaw;
    float gyroX;
    float gyroY;
    float gyroZ;
};

void initEstimator();
void calibrateEstimator();
AttitudeData updateEstimator(float dt);
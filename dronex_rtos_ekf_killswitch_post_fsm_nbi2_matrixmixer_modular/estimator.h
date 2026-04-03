#pragma once
#include <Arduino.h>

struct AttitudeData {
    float pitch;
    float roll;
    float yaw;
    float gyroX;
    float gyroY;
    float gyroZ;
};

struct RawSensorData {
    int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyX, rawGyY, rawGyZ;
};

extern RawSensorData latest_sensor_data;
extern portMUX_TYPE sensor_mux; 

void initEstimator();
void calibrateEstimator();
AttitudeData updateEstimator(float dt);

void Task_SensorRead(void *pvParameters);
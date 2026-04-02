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

// Struct for passing data between tasks safely
struct RawSensorData {
    int16_t rawAccX, rawAccY, rawAccZ;
    int16_t rawGyX, rawGyY, rawGyZ;
};

// REMOVED 'volatile' here because the mutex memory barrier handles synchronization
extern RawSensorData latest_sensor_data;
extern portMUX_TYPE sensor_mux; // Mutex for thread safety

void initEstimator();
void calibrateEstimator();
AttitudeData updateEstimator(float dt);

// Task definition for non-blocking reads
void Task_SensorRead(void *pvParameters);

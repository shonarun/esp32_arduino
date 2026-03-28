#pragma once
#include <Arduino.h>
#include <Wire.h>

void Sensors_Init();
void Sensors_Calibrate();
void Sensors_Read(float &gyroRadX, float &gyroRadY, float &gyroRadZ, 
                  float &accX, float &accY, float &accZ, 
                  float &gyroRateX, float &gyroRateY, float &gyroRateZ);
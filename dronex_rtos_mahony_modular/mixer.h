#pragma once
#include <Arduino.h>

// Hardware Pins
#define PIN_MOTOR_FL 4  // Front Left
#define PIN_MOTOR_FR 5  // Front Right
#define PIN_MOTOR_BL 3  // Back Left
#define PIN_MOTOR_BR 6  // Back Right

void Mixer_Init();
void Mixer_Stop();
void Mixer_Write(int base_thrust, float pitch_pid, float roll_pid, float yaw_pid);

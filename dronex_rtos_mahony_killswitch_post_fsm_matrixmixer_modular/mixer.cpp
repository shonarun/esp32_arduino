#include "mixer.h"
#include "config.h"

// ---------------------------------------------------------
// CUSTOM GEOMETRY MATRIX
// Replace these values with the normalized distances from 
// your center of gravity to the motors.
// Format: { Thrust, Roll, Pitch, Yaw }
// ---------------------------------------------------------
const float MIX_MATRIX[4][4] = {
    // FL Motor
    { 1.0f,  1.0f,  1.0f,  1.0f }, 
    // FR Motor
    { 1.0f, -1.0f,  1.0f, -1.0f },
    // BL Motor
    { 1.0f,  1.0f, -1.0f, -1.0f },
    // BR Motor
    { 1.0f, -1.0f, -1.0f,  1.0f } 
};

void initMixer() {
  ledcAttach(PIN_MOTOR_FL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_FR, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BR, MOT_FREQ, PWM_RESN);
  killMotors();
}

void killMotors() {
  ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
}

void applyMotorMixing(int base_thrust, PID_Output pid) {
    float pid_vector[4] = { 
        (float)base_thrust, 
        pid.roll, 
        pid.pitch, 
        pid.yaw 
    };

    float motor_out[4] = {0, 0, 0, 0};
    float max_motor = -9999.0f;
    float min_motor = 9999.0f;

    // 1. Matrix Vector Multiplication (Calculate raw mix)
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            motor_out[i] += MIX_MATRIX[i][j] * pid_vector[j];
        }
        // Track the highest and lowest motor demands
        if (motor_out[i] > max_motor) max_motor = motor_out[i];
        if (motor_out[i] < min_motor) min_motor = motor_out[i];
    }

    // 2. AirMode: Prevent Motor Stalls (Death Roll Prevention)
    // If the PID loop asks a motor to go below idle, we instead boost ALL motors.
    if (min_motor < MOTOR_MIN_IDLE) {
        float boost = MOTOR_MIN_IDLE - min_motor;
        for (int i = 0; i < 4; i++) {
            motor_out[i] += boost;
        }
        max_motor += boost; // Update the new maximum after the boost
    }

    // 3. Desaturation: Prevent Top-End Clipping (Preserve Ratios)
    // If pushing high throttle/boost pushes a motor past the max PWM limit,
    // we SCALE the outputs down towards the idle baseline.
    if (max_motor > MOTOR_MAX_PWM) {
        // Calculate the scaling factor needed to squeeze the highest motor under the max limit
        float scale = (MOTOR_MAX_PWM - MOTOR_MIN_IDLE) / (max_motor - MOTOR_MIN_IDLE);
        for (int i = 0; i < 4; i++) {
            motor_out[i] = ((motor_out[i] - MOTOR_MIN_IDLE) * scale) + MOTOR_MIN_IDLE;
        }
    }

    // 4. Write to Hardware (No need to constrain anymore, the math guarantees it fits)
    ledcWrite(PIN_MOTOR_FL, (int)motor_out[0]);
    ledcWrite(PIN_MOTOR_FR, (int)motor_out[1]);
    ledcWrite(PIN_MOTOR_BL, (int)motor_out[2]);
    ledcWrite(PIN_MOTOR_BR, (int)motor_out[3]);
}
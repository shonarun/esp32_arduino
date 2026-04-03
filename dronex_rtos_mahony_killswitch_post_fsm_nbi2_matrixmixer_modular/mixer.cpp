#include "mixer.h"
#include "config.h"

// ---------------------------------------------------------
// CUSTOM GEOMETRY MATRIX
// Format: { Thrust, Roll, Pitch, Yaw }
// ---------------------------------------------------------
const float MIX_MATRIX[4][4] = {
    { 1.0f,  1.0f,  1.0f,  1.0f }, // FL Motor
    
    { 1.0f, -1.0f,  1.0f, -1.0f }, // FR Motor

    { 1.0f,  1.0f, -1.0f, -1.0f }, // BL Motor

    { 1.0f, -1.0f, -1.0f,  1.0f }  // BR Motor
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

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            motor_out[i] += MIX_MATRIX[i][j] * pid_vector[j];
        }
        if (motor_out[i] > max_motor) max_motor = motor_out[i];
        if (motor_out[i] < min_motor) min_motor = motor_out[i];
    }

    if (min_motor < MOTOR_MIN_IDLE) {
        float boost = MOTOR_MIN_IDLE - min_motor;
        for (int i = 0; i < 4; i++) {
            motor_out[i] += boost;
        }
        max_motor += boost; 
    }

    if (max_motor > MOTOR_MAX_PWM) {
        float scale = (MOTOR_MAX_PWM - MOTOR_MIN_IDLE) / (max_motor - MOTOR_MIN_IDLE);
        for (int i = 0; i < 4; i++) {
            motor_out[i] = ((motor_out[i] - MOTOR_MIN_IDLE) * scale) + MOTOR_MIN_IDLE;
        }
    }

    ledcWrite(PIN_MOTOR_FL, (int)motor_out[0]);
    ledcWrite(PIN_MOTOR_FR, (int)motor_out[1]);
    ledcWrite(PIN_MOTOR_BL, (int)motor_out[2]);
    ledcWrite(PIN_MOTOR_BR, (int)motor_out[3]);
}
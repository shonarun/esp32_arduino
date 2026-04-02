#include "mixer.h"
#include "config.h"

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
    int raw_FL = base_thrust + pid.pitch + pid.roll + pid.yaw;
    int raw_FR = base_thrust + pid.pitch - pid.roll - pid.yaw;
    int raw_BL = base_thrust - pid.pitch + pid.roll - pid.yaw;
    int raw_BR = base_thrust - pid.pitch - pid.roll + pid.yaw;

    ledcWrite(PIN_MOTOR_FL, constrain(raw_FL, 0, 800));
    ledcWrite(PIN_MOTOR_FR, constrain(raw_FR, 0, 800));
    ledcWrite(PIN_MOTOR_BL, constrain(raw_BL, 0, 800));
    ledcWrite(PIN_MOTOR_BR, constrain(raw_BR, 0, 800));
}
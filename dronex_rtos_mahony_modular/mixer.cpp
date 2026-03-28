#include "mixer.h"

#define MOT_FREQ 20000
#define PWM_RESN 10

void Mixer_Init() {
  ledcAttach(PIN_MOTOR_FL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_FR, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BR, MOT_FREQ, PWM_RESN);
  
  Mixer_Stop();
}

void Mixer_Stop() {
  ledcWrite(PIN_MOTOR_FL, 0);
  ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0);
  ledcWrite(PIN_MOTOR_BR, 0);
}

void Mixer_Write(int base_thrust, float pitch_pid, float roll_pid, float yaw_pid) {
  int speed_FL = constrain(base_thrust + pitch_pid + roll_pid + yaw_pid, 0, 800);
  int speed_FR = constrain(base_thrust + pitch_pid - roll_pid - yaw_pid, 0, 800);
  int speed_BL = constrain(base_thrust - pitch_pid + roll_pid - yaw_pid, 0, 800);
  int speed_BR = constrain(base_thrust - pitch_pid - roll_pid + yaw_pid, 0, 800);

  ledcWrite(PIN_MOTOR_FL, speed_FL);
  ledcWrite(PIN_MOTOR_FR, speed_FR);
  ledcWrite(PIN_MOTOR_BL, speed_BL);
  ledcWrite(PIN_MOTOR_BR, speed_BR);
}
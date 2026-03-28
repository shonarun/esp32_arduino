#include "pid.h"

// Gains
float Kp_angle = 2.0; 

float Kp_rate = 1.0, Ki_rate = 0.0, Kd_rate = 0.001;
float Kp_yaw_rate = 1.5, Ki_yaw_rate = 0.0, Kd_yaw_rate = 0.0;

// State
float pitch_rate_integral = 0, roll_rate_integral = 0, yaw_rate_integral = 0;
float pitch_prev_rate_error = 0, roll_prev_rate_error = 0, yaw_prev_rate_error = 0;

void PID_Reset() {
  pitch_rate_integral = 0;
  roll_rate_integral = 0;
  yaw_rate_integral = 0;
  pitch_prev_rate_error = 0;
  roll_prev_rate_error = 0;
  yaw_prev_rate_error = 0;
}

void PID_Compute(float target_pitch, float target_roll, float target_yaw_rate,
                 float actual_pitch, float actual_roll,
                 float gyroRateX, float gyroRateY, float gyroRateZ,
                 float dt, 
                 float &pitch_out, float &roll_out, float &yaw_out) {
                 
  // OUTER LOOP: Angle PID
  float pitch_angle_error = target_pitch - actual_pitch;
  float roll_angle_error  = target_roll  - actual_roll;

  float desired_pitch_rate = Kp_angle * pitch_angle_error; 
  float desired_roll_rate  = Kp_angle * roll_angle_error;

  // INNER LOOP: Rate PID 
  float pitch_rate_error = desired_pitch_rate - gyroRateY;
  float roll_rate_error  = desired_roll_rate  - gyroRateX;
  float yaw_rate_error   = target_yaw_rate - gyroRateZ;

  pitch_rate_integral += pitch_rate_error * dt;
  roll_rate_integral  += roll_rate_error  * dt;
  yaw_rate_integral   += yaw_rate_error   * dt;

  pitch_rate_integral = constrain(pitch_rate_integral, -400.0, 400.0);
  roll_rate_integral  = constrain(roll_rate_integral,  -400.0, 400.0);
  yaw_rate_integral   = constrain(yaw_rate_integral,   -400.0, 400.0);

  pitch_out = (Kp_rate * pitch_rate_error) + (Ki_rate * pitch_rate_integral) + (Kd_rate * (pitch_rate_error - pitch_prev_rate_error) / dt);
  roll_out  = (Kp_rate * roll_rate_error)  + (Ki_rate * roll_rate_integral)  + (Kd_rate * (roll_rate_error - roll_prev_rate_error) / dt);
  yaw_out   = (Kp_yaw_rate * yaw_rate_error) + (Ki_yaw_rate * yaw_rate_integral) + (Kd_yaw_rate * (yaw_rate_error - yaw_prev_rate_error) / dt);

  pitch_prev_rate_error = pitch_rate_error;
  roll_prev_rate_error  = roll_rate_error;
  yaw_prev_rate_error   = yaw_rate_error;
}
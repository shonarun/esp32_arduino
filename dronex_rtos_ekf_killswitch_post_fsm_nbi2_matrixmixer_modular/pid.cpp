#include "pid.h"
#include "config.h"

static float pitch_integral = 0, roll_integral = 0, yaw_integral = 0;
static float prev_gyroY = 0, prev_gyroX = 0, prev_gyroZ = 0;

void resetPID() {
    pitch_integral = 0; roll_integral = 0; yaw_integral = 0;
}

PID_Output calculatePID(AttitudeData current_attitude, float dt) {
    PID_Output output;

    float desired_pitch_rate = Kp_angle * (target_pitch - current_attitude.pitch); 
    float desired_roll_rate  = Kp_angle * (target_roll  - current_attitude.roll);

    float pitch_error = desired_pitch_rate - current_attitude.gyroY;
    float roll_error  = desired_roll_rate  - current_attitude.gyroX;
    float yaw_error   = target_yaw_rate    - current_attitude.gyroZ; 

    pitch_integral = constrain(pitch_integral + (pitch_error * dt), -400.0, 400.0);
    roll_integral  = constrain(roll_integral + (roll_error * dt),  -400.0, 400.0);
    yaw_integral   = constrain(yaw_integral + (yaw_error * dt),   -400.0, 400.0);

    output.pitch = (Kp_rate * pitch_error) + (Ki_rate * pitch_integral) - (Kd_rate * (current_attitude.gyroY - prev_gyroY) / dt);
    output.roll  = (Kp_rate * roll_error)  + (Ki_rate * roll_integral)  - (Kd_rate * (current_attitude.gyroX - prev_gyroX) / dt);
    output.yaw   = (Kp_yaw_rate * yaw_error) + (Ki_yaw_rate * yaw_integral) - (Kd_yaw_rate * (current_attitude.gyroZ - prev_gyroZ) / dt);
    
    prev_gyroY = current_attitude.gyroY; prev_gyroX = current_attitude.gyroX; prev_gyroZ = current_attitude.gyroZ;
    return output;
}
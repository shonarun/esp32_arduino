#include "config.h"

volatile int current_base_thrust = 0;
volatile float target_pitch = 0.0;
volatile float target_roll  = 0.0;
volatile float target_yaw_rate = 0.0;

float Kp_angle = 2.0; 
float Kp_rate = 1.0, Ki_rate = 0.0, Kd_rate = 0.001;
float Kp_yaw_rate = 1.5, Ki_yaw_rate = 0.0, Kd_yaw_rate = 0.0;

TaskHandle_t FlightControlTask;
TaskHandle_t CommanderTask;
TaskHandle_t CommsTask;
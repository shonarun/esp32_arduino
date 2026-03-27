#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Arduino.h>

#define MOT_FREQ 20000
#define PWM_RESN 10

#define MPU_FILTER_CONFIG 0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C

// ==========================================
// LITEWING v1.2 HARDWARE PINS (ESP32-S3)
// ==========================================
#define PIN_MOTOR_FL 4  // Front Left
#define PIN_MOTOR_FR 5  // Front Right
#define PIN_MOTOR_BL 3  // Back Left
#define PIN_MOTOR_BR 6  // Back Right

#define I2C_SDA 11      // MPU6050 SDA
#define I2C_SCL 10      // MPU6050 SCL

const uint8_t MPU_ADDR = 0x68;
const int16_t ACCEL_OFFSET_X = 1000; // -83, -367
const int16_t ACCEL_OFFSET_Y = -700; // -74,  235
const int16_t ACCEL_OFFSET_Z = 1160; // -28,  890

// ==========================================
// RTOS TASK HANDLES & SHARED VARIABLES
// ==========================================
TaskHandle_t FlightControlTask;
TaskHandle_t CommanderTask;

volatile int current_base_thrust = 0;
volatile bool is_flying = false;
volatile bool calibrate_done = false;

volatile float target_pitch = 0.0;
volatile float target_roll  = 0.0;
volatile float target_yaw_rate = 0.0; 

// ==========================================
// FLIGHT STATE MACHINE
// ==========================================
enum FlightState {
  STATE_BOOT,
  STATE_STANDBY,
  STATE_ARMED_TAKEOFF,
  STATE_FLYING,
  STATE_KILLED
};

FlightState current_state = STATE_BOOT;

// ==========================================
// FLIGHT SETTINGS & CASCADED PID
// ==========================================
const int HOVER_THRUST = 511; 
const float LOOP_TIME_SEC = 0.004; // 4ms loop = 250Hz
const TickType_t LOOP_TICKS = pdMS_TO_TICKS(4); 

// Outer Loop (Angle)
float Kp_angle = 2.0; 

// Inner Loop (Rate)
float Kp_rate = 1.0, Ki_rate = 0.0, Kd_rate = 0.001;
float Kp_yaw_rate = 1.5, Ki_yaw_rate = 0.0, Kd_yaw_rate = 0.0;
float pitch_rate_integral = 0, roll_rate_integral = 0, yaw_rate_integral = 0;
float prev_gyroRateX = 0, prev_gyroRateY = 0, prev_gyroRateZ = 0;

float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;
float actual_pitch = 0.0, actual_roll  = 0.0, actual_yaw = 0.0;

// ==========================================
// KALMAN FILTER GLOBALS
// ==========================================
// Tuning parameters
const float Q_angle = 0.001f;   // Process noise variance for the angle
const float Q_bias = 0.003f;    // Process noise variance for the gyro bias
const float R_measure = 0.03f;  // Measurement noise variance (accelerometer)

// Roll Kalman State
float roll_bias = 0.0f;
float P_roll[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

// Pitch Kalman State
float pitch_bias = 0.0f;
float P_pitch[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

// --- New Safety Globals ---
volatile bool emergency_kill = true; // STARTS TRUE FOR SAFETY
volatile bool is_armed = false;
TaskHandle_t CommsTask;
WebServer server(80);

// Function Prototypes
void calibrateGyro();
void Task_FlightControl(void *pvParameters);
void Task_Commander(void *pvParameters);
void Kalman_Update(float gyroRateX, float gyroRateY, float gyroRateZ, float accX, float accY, float accZ, float dt);
void Task_Comms(void *pvParameters);

// --- Secure HTML Interface ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin-top: 50px; background-color: #222; color: white;}
    .btn { display: block; width: 80%; margin: 20px auto; padding: 30px; font-size: 30px; font-weight: bold; border-radius: 10px; border: none; color: white; cursor: pointer; }
    .btn-arm { background-color: #28a745; }
    .btn-kill { background-color: #dc3545; padding: 50px; font-size: 40px;}
    .btn:active { opacity: 0.7; }
    #status { font-size: 20px; margin-top: 20px; font-weight: bold; color: #ffc107; }
  </style>
</head>
<body>
  <h1>LITEWING v1.2</h1>
  <div id="status">STATUS: AWAITING COMMAND</div>
  
  <button class="btn btn-kill" onclick="sendCommand('/kill', 'KILLED', '#dc3545')">EMERGENCY KILL</button>
  <button class="btn btn-arm" onclick="sendCommand('/arm', 'ARMED', '#28a745')">ARM DRONE</button>

  <script>
    function sendCommand(url, statusText, statusColor) {
      if (navigator.vibrate) navigator.vibrate(200); // Vibrate phone on press
      
      // Explicitly use POST to prevent browser pre-fetching
      fetch(url, { method: 'POST' })
        .then(response => {
          if (response.ok) {
            const statusEl = document.getElementById('status');
            statusEl.innerText = 'STATUS: ' + statusText;
            statusEl.style.color = statusColor;
          }
        })
        .catch(err => console.error('Command failed to send:', err));
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  
  // Initialize Motors
  ledcAttach(PIN_MOTOR_FL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_FR, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BL, MOT_FREQ, PWM_RESN);
  ledcAttach(PIN_MOTOR_BR, MOT_FREQ, PWM_RESN);
  
  ledcWrite(PIN_MOTOR_FL, 0);
  ledcWrite(PIN_MOTOR_FR, 0);
  ledcWrite(PIN_MOTOR_BL, 0);
  ledcWrite(PIN_MOTOR_BR, 0);

  // Initialize the MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Wake up
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_FILTER_CONFIG); // DLPF
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CONFIG); // Gyro +/- 500 deg/s
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CONFIG); // Accel +/- 4g
  Wire.write(0x08);
  Wire.endTransmission();

  xTaskCreatePinnedToCore(Task_Comms, "CommsTask", 4096, NULL, 1, &CommsTask, 0);
  xTaskCreatePinnedToCore(Task_FlightControl, "FlightControl", 4096, NULL, 2, &FlightControlTask, 1);
  xTaskCreatePinnedToCore(Task_Commander, "Commander", 4096, NULL, 1, &CommanderTask, 0);
}

void loop() {
  vTaskDelete(NULL); 
}

// ==========================================
// TASK 1: STRICT TIMING CONTROL LOOP (Core 1)
// ==========================================
void Task_FlightControl(void *pvParameters) {
  TickType_t xLastWakeTime;
  
  while(!calibrate_done) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, LOOP_TICKS);

    int16_t accX, accY, accZ, tempRaw, gyroX, gyroY, gyroZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (bytesReceived != 14) continue;

    int16_t rawAccX = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_X;
    int16_t rawAccY = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Y;
    int16_t rawAccZ = (Wire.read() << 8 | Wire.read()) + ACCEL_OFFSET_Z;
    tempRaw         = (Wire.read() << 8 | Wire.read());
    int16_t rawGyX  = (Wire.read() << 8 | Wire.read());
    int16_t rawGyY  = (Wire.read() << 8 | Wire.read());
    int16_t rawGyZ  = (Wire.read() << 8 | Wire.read());
    
    accX  = -rawAccY;
    accY  = -rawAccX;
    accZ  = rawAccZ;
    
    gyroX = -rawGyY;
    gyroY = -rawGyX;
    gyroZ = rawGyZ;

    // Gyro Rates (Degrees/sec)
    float gyroRateX = (gyroX / 65.5) - gyroErrorX;
    float gyroRateY = (gyroY / 65.5) - gyroErrorY;
    float gyroRateZ = (gyroZ / 65.5) - gyroErrorZ;

    static unsigned long prev_time = 0;
    unsigned long current_time = micros();
    float dt = (current_time - prev_time) / 1000000.0f;
    prev_time = current_time;

    // Sensor Fusion Update via Kalman
    Kalman_Update(gyroRateX, gyroRateY, gyroRateZ, (float)accX, (float)accY, (float)accZ, dt);
    // Kalman_Update(gyroRateX, gyroRateY, gyroRateZ, (float)accX, (float)accY, (float)accZ, LOOP_TIME_SEC);

    // --- EMERGENCY KILL OVERRIDE ---
    if (emergency_kill || !is_flying) {
      pitch_rate_integral = 0; roll_rate_integral = 0; yaw_rate_integral = 0;
      ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
      ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
      continue; // Skip the rest of the PID loop
    }

    float local_target_pitch = target_pitch;
    float local_target_roll  = target_roll;
    float local_target_yaw_rate = target_yaw_rate;
    int base_t = current_base_thrust; 

    // OUTER LOOP: Angle PID
    float pitch_angle_error = local_target_pitch - actual_pitch;
    float roll_angle_error  = local_target_roll  - actual_roll;

    float desired_pitch_rate = Kp_angle * pitch_angle_error; 
    float desired_roll_rate  = Kp_angle * roll_angle_error;

    // INNER LOOP: Rate PID 
    float pitch_rate_error = desired_pitch_rate - gyroRateY;
    float roll_rate_error  = desired_roll_rate  - gyroRateX;
    float yaw_rate_error   = local_target_yaw_rate - gyroRateZ; 

    pitch_rate_integral += pitch_rate_error * dt;
    roll_rate_integral  += roll_rate_error  * dt;
    yaw_rate_integral   += yaw_rate_error   * dt;

    // pitch_rate_integral += pitch_rate_error * LOOP_TIME_SEC;
    // roll_rate_integral  += roll_rate_error  * LOOP_TIME_SEC;
    // yaw_rate_integral   += yaw_rate_error   * LOOP_TIME_SEC;

    pitch_rate_integral = constrain(pitch_rate_integral, -400.0, 400.0);
    roll_rate_integral  = constrain(roll_rate_integral,  -400.0, 400.0);
    yaw_rate_integral   = constrain(yaw_rate_integral,   -400.0, 400.0);

    // --- NEW: Derivative on Measurement ---
    // Note the negative sign at the start of the D-term!
    float pitch_pid_output = (Kp_rate * pitch_rate_error) + (Ki_rate * pitch_rate_integral) - (Kd_rate * (gyroRateY - prev_gyroRateY) / dt);
    float roll_pid_output  = (Kp_rate * roll_rate_error)  + (Ki_rate * roll_rate_integral)  - (Kd_rate * (gyroRateX - prev_gyroRateX) / dt);
    float yaw_pid_output   = (Kp_yaw_rate * yaw_rate_error) + (Ki_yaw_rate * yaw_rate_integral) - (Kd_yaw_rate * (gyroRateZ - prev_gyroRateZ) / dt);
    
    // float pitch_pid_output = (Kp_rate * pitch_rate_error) + (Ki_rate * pitch_rate_integral) - (Kd_rate * (gyroRateY - prev_gyroRateY) / LOOP_TIME_SEC);
    // float roll_pid_output  = (Kp_rate * roll_rate_error)  + (Ki_rate * roll_rate_integral)  - (Kd_rate * (gyroRateX - prev_gyroRateX) / LOOP_TIME_SEC);
    // float yaw_pid_output   = (Kp_yaw_rate * yaw_rate_error) + (Ki_yaw_rate * yaw_rate_integral) - (Kd_yaw_rate * (gyroRateZ - prev_gyroRateZ) / LOOP_TIME_SEC);

    // --- NEW: Store current gyro rates for the next loop ---
    prev_gyroRateY = gyroRateY;
    prev_gyroRateX = gyroRateX;
    prev_gyroRateZ = gyroRateZ;

    // pitch_prev_rate_error = pitch_rate_error;
    // roll_prev_rate_error  = roll_rate_error;
    // yaw_prev_rate_error   = yaw_rate_error;

    // Motor Mixing
    int raw_FL = base_t + pitch_pid_output + roll_pid_output + yaw_pid_output;
    int raw_FR = base_t + pitch_pid_output - roll_pid_output - yaw_pid_output;
    int raw_BL = base_t - pitch_pid_output + roll_pid_output - yaw_pid_output;
    int raw_BR = base_t - pitch_pid_output - roll_pid_output + yaw_pid_output;

    int speed_FL = constrain(raw_FL, 0, 800);
    int speed_FR = constrain(raw_FR, 0, 800);
    int speed_BL = constrain(raw_BL, 0, 800);
    int speed_BR = constrain(raw_BR, 0, 800);

    ledcWrite(PIN_MOTOR_FL, speed_FL);
    ledcWrite(PIN_MOTOR_FR, speed_FR);
    ledcWrite(PIN_MOTOR_BL, speed_BL);
    ledcWrite(PIN_MOTOR_BR, speed_BR);
  }
}

// ==========================================
// TASK 2: FLIGHT COMMANDER (Core 0)
// ==========================================
void Task_Commander(void *pvParameters) {
  // --- 1. ONE-TIME BOOT SEQUENCE ---
  Serial.println("Drone booted. Waiting 8 seconds...");
  vTaskDelay(pdMS_TO_TICKS(8000)); // Non-blocking wait
  
  calibrateGyro();
  calibrate_done = true; 
  Serial.println("Gyro Calibrated...");

  unsigned long state_timer = 0; // Used for timing takeoff and flight

  // --- 2. MAIN STATE MACHINE LOOP ---
  for (;;) {
    // [SAFETY OVERRIDE] - Always check for a kill command first
    if (emergency_kill && current_state != STATE_KILLED && current_state != STATE_BOOT) {
      current_state = STATE_KILLED;
      is_flying = false;
      is_armed = false;
      current_base_thrust = 0;
      Serial.println("FSM: EMERGENCY KILL TRIGGERED!");
    }

    switch (current_state) {
      
      case STATE_BOOT:
        Serial.println("STATUS: STANDBY. Waiting for ARM command from Web UI...");
        current_state = STATE_STANDBY;
        break;

      case STATE_STANDBY:
        // Wait here until the Web UI sends the /arm command
        if (is_armed && !emergency_kill) {
          Serial.println("STATUS: ARMED! Taking off in 3 seconds...");
          state_timer = millis(); // Record the exact time we armed
          current_state = STATE_ARMED_TAKEOFF;
        }
        break;

      case STATE_ARMED_TAKEOFF:
        // Non-blocking 3-second delay
        if (millis() - state_timer >= 3000) {
          Serial.println("STATUS: FLYING! Starting sequence.");
          state_timer = millis(); // Reset timer to track time-in-air
          target_pitch = 0.0;
          target_roll = 0.0;
          is_flying = true;
          current_state = STATE_FLYING;
        }
        break;

      case STATE_FLYING:
        {
          unsigned long time_in_air = millis() - state_timer;

          if (time_in_air < 1000) {
            // Spool up smoothly
            current_base_thrust = map(time_in_air, 0, 1000, 0, HOVER_THRUST);
          } 
          else if (time_in_air < 6000) {
            // Hold hover
            current_base_thrust = HOVER_THRUST;
          } 
          else {
            // Sequence complete. Auto-land/shutdown.
            Serial.println("STATUS: Flight Sequence Complete. Motors off.");
            is_flying = false;
            current_base_thrust = 0;
            
            ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
            ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
            
            is_armed = false; // Disarm the drone
            current_state = STATE_BOOT; // Cycle back to the beginning!
          }
        }
        break;

      case STATE_KILLED:
        // Drone is dead in the water. We wait here until the user 
        // hits "ARM" on the web UI, which clears the emergency_kill flag.
        if (!emergency_kill) {
          Serial.println("STATUS: Kill switch cleared. Returning to Standby.");
          // We reset to BOOT to trigger the standby print message again
          current_state = STATE_BOOT; 
        }
        break;
    }

    // Yield to FreeRTOS. 20ms = 50Hz Commander Loop
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

// ==========================================
// HELPER FUNCTIONS & SENSOR FUSION
// ==========================================
void calibrateGyro() {
  gyroErrorX = 0; gyroErrorY = 0; gyroErrorZ = 0;
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    
    int16_t gX = (Wire.read() << 8 | Wire.read());
    int16_t gY = (Wire.read() << 8 | Wire.read());
    int16_t gZ = (Wire.read() << 8 | Wire.read());
    
    gyroErrorX += (gX / 65.5); 
    gyroErrorY += (gY / 65.5);
    gyroErrorZ += (gZ / 65.5);
    
    vTaskDelay(pdMS_TO_TICKS(3)); 
  }
  gyroErrorX /= 2000; gyroErrorY /= 2000; gyroErrorZ /= 2000;
}

// 2-State Kalman Filter for Pitch and Roll
void Kalman_Update(float gyroRateX, float gyroRateY, float gyroRateZ, float accX, float accY, float accZ, float dt) {
    
    // 1. Calculate Pitch and Roll from Accelerometer
    // atan2 outputs radians, convert to degrees
    float accRoll = atan2(accY, accZ) * 57.2958f;
    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 57.2958f;

    // --- ROLL KALMAN FILTER ---
    // Step 1: Predict state
    float rate_roll = gyroRateX - roll_bias;
    actual_roll += dt * rate_roll;

    // Step 2: Predict error covariance
    P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
    P_roll[0][1] -= dt * P_roll[1][1];
    P_roll[1][0] -= dt * P_roll[1][1];
    P_roll[1][1] += Q_bias * dt;

    // Step 3: Calculate Kalman gain
    float S_roll = P_roll[0][0] + R_measure;
    float K_roll[2];
    K_roll[0] = P_roll[0][0] / S_roll;
    K_roll[1] = P_roll[1][0] / S_roll;

    // Step 4: Update state with measurement
    float y_roll = accRoll - actual_roll;
    actual_roll += K_roll[0] * y_roll;
    roll_bias += K_roll[1] * y_roll;

    // Step 5: Update error covariance
    float P00_temp_r = P_roll[0][0];
    float P01_temp_r = P_roll[0][1];
    P_roll[0][0] -= K_roll[0] * P00_temp_r;
    P_roll[0][1] -= K_roll[0] * P01_temp_r;
    P_roll[1][0] -= K_roll[1] * P00_temp_r;
    P_roll[1][1] -= K_roll[1] * P01_temp_r;


    // --- PITCH KALMAN FILTER ---
    // Step 1: Predict state
    float rate_pitch = gyroRateY - pitch_bias;
    actual_pitch += dt * rate_pitch;

    // Step 2: Predict error covariance
    P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
    P_pitch[0][1] -= dt * P_pitch[1][1];
    P_pitch[1][0] -= dt * P_pitch[1][1];
    P_pitch[1][1] += Q_bias * dt;

    // Step 3: Calculate Kalman gain
    float S_pitch = P_pitch[0][0] + R_measure;
    float K_pitch[2];
    K_pitch[0] = P_pitch[0][0] / S_pitch;
    K_pitch[1] = P_pitch[1][0] / S_pitch;

    // Step 4: Update state with measurement
    float y_pitch = accPitch - actual_pitch;
    actual_pitch += K_pitch[0] * y_pitch;
    pitch_bias += K_pitch[1] * y_pitch;

    // Step 5: Update error covariance
    float P00_temp_p = P_pitch[0][0];
    float P01_temp_p = P_pitch[0][1];
    P_pitch[0][0] -= K_pitch[0] * P00_temp_p;
    P_pitch[0][1] -= K_pitch[0] * P01_temp_p;
    P_pitch[1][0] -= K_pitch[1] * P00_temp_p;
    P_pitch[1][1] -= K_pitch[1] * P01_temp_p;

    // --- YAW UPDATE ---
    // Without a magnetometer, we can only integrate the Z gyro
    actual_yaw += gyroRateZ * dt; 
}

void Task_Comms(void *pvParameters) {
  // 1. Start WiFi Access Point
  WiFi.softAP("LITEWING_DRONE", "12345678"); // Network Name and Password
  
  // 2. Define Web Server Routes
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", INDEX_HTML);
  });

  // Switched to HTTP_POST to prevent accidental pre-fetching
  server.on("/arm", HTTP_POST, []() {
    emergency_kill = false;
    is_armed = true;
    Serial.println("WEB: DRONE ARMED!");
    server.send(200, "text/plain", "Armed");
  });

  // Switched to HTTP_POST to prevent accidental pre-fetching
  server.on("/kill", HTTP_POST, []() {
    emergency_kill = true;
    is_armed = false;
    is_flying = false;
    Serial.println("WEB: EMERGENCY KILL ENGAGED!");
    // Instantly kill motors bypassing the control loop for safety
    ledcWrite(PIN_MOTOR_FL, 0); ledcWrite(PIN_MOTOR_FR, 0);
    ledcWrite(PIN_MOTOR_BL, 0); ledcWrite(PIN_MOTOR_BR, 0);
    server.send(200, "text/plain", "Killed");
  });

  server.begin();
  Serial.println("Web Server Started on 192.168.4.1");

  // 3. Keep listening indefinitely
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to let Core 0 breathe
  }
}

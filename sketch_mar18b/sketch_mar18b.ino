#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int I2C_SDA = 11;      // MPU6050 SDA
const int I2C_SCL = 10;      // MPU6050 SCL

float actual_pitch = 0.0;
float actual_roll = 0.0;
float mag = 0.0;

float sum_p = 0.0;
float sum_r = 0.0;

int samples = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! Check I2C pins.");
    while (1) { delay(10); } // Halt if sensor isn't found
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Drone booted. Waiting 10 seconds for safety...");
  
  for (int i = 0; i < samples; i++) {
    // --- Read IMU Data ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    actual_pitch = atan2(-a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;
    actual_roll = atan2(a.acceleration.x,sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
    mag = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
    
    sum_p += actual_pitch;
    sum_r += actual_roll;
    Serial.print("count: ");
    Serial.print(i);
    Serial.print("actual_pitch: ");
    Serial.print(actual_pitch);
    Serial.print(", actual_roll: ");
    Serial.print(actual_roll);
    Serial.print(", mag: ");
    Serial.print(mag);
    Serial.print(", ax: ");
    Serial.print(a.acceleration.x);
    Serial.print(", ay: ");
    Serial.print(a.acceleration.y);
    Serial.print(", az: ");
    Serial.println(a.acceleration.z);
    delay(100);
  }


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("mean_pitch: ");
  Serial.print(sum_p / samples);
  Serial.print(", mean_roll: ");
  Serial.println(sum_r / samples);
  delay(1000);
}

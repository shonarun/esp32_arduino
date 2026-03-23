// Tumble
#include <Wire.h>

// ESP32-S3 specific pins
#define I2C_SDA 11
#define I2C_SCL 10
const uint8_t MPU_ADDR = 0x68; 

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set Accel to ±8g (1g = 4096 raw)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); 
  Wire.write(0x10); 
  Wire.endTransmission(true);

  // Countdown
  Serial.println("\n--- MPU6050 TUMBLE CALIBRATION ---");
  Serial.println("Starting in 5 seconds...");
  for (int i = 5; i > 0; i--) {
    Serial.printf("%d...\n", i);
    delay(1000);
  }
  Serial.println("GO! Slowly tumble the drone in all 6 directions...");

  // Initialize tracking variables
  int16_t minX = 32767, maxX = -32768;
  int16_t minY = 32767, maxY = -32768;
  int16_t minZ = 32767, maxZ = -32768;

  // Run the tumble loop for 30 seconds (30,000 milliseconds)
  uint32_t startTime = millis();
  
  while (millis() - startTime < 30000) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);
    
    int16_t aX = (Wire.read() << 8 | Wire.read());
    int16_t aY = (Wire.read() << 8 | Wire.read());
    int16_t aZ = (Wire.read() << 8 | Wire.read());
    
    // Track extremes
    if (aX < minX) minX = aX; if (aX > maxX) maxX = aX;
    if (aY < minY) minY = aY; if (aY > maxY) maxY = aY;
    if (aZ < minZ) minZ = aZ; if (aZ > maxZ) maxZ = aZ;
    
    // Print a dot every half second so you know it's working
    if ((millis() - startTime) % 500 < 5) {
      Serial.print(".");
      delay(5); // Prevent multiple dots in the same millisecond window
    }
  }

  Serial.println("\n\nDone! Calculating offsets...");

  // The center point (bias) is the exact middle of the min and max.
  // We flip the sign (-) because we want the correction offset to ADD to the raw data.
  int16_t offsetX = -((maxX + minX) / 2);
  int16_t offsetY = -((maxY + minY) / 2);
  int16_t offsetZ = -((maxZ + minZ) / 2);

  Serial.println("--- HARDCODE THESE INTO YOUR MAIN FLIGHT LOOP ---");
  Serial.printf("const int16_t ACCEL_OFFSET_X = %d;\n", offsetX);
  Serial.printf("const int16_t ACCEL_OFFSET_Y = %d;\n", offsetY);
  Serial.printf("const int16_t ACCEL_OFFSET_Z = %d;\n", offsetZ);
  Serial.println("-------------------------------------------------");
}

void loop() {
  // Calibration complete, nothing else to do.
}
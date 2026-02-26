#include <Wire.h>

const int MPU_ADDR = 0x68;
float gyroZ_offset = 0;
float angleZ = 0;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Calibrating... Keep still!");

  // Calibrate
  float sum = 0;
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(1);
  }
  gyroZ_offset = (sum / 1000.0) / 131.0;

  Serial.println("Done! Rotate your robot and watch the angle.");
  Serial.println("-------------------------------------------");
  previousTime = millis();
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t raw = Wire.read() << 8 | Wire.read();
  float gyroZ = (raw / 131.0) - gyroZ_offset;
  if (abs(gyroZ) < 0.15) gyroZ = 0;

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  angleZ += gyroZ * dt;

  Serial.print("GyroZ: ");   Serial.print(gyroZ,  2);
  Serial.print(" deg/s  |  Angle: "); Serial.print(angleZ, 2);
  Serial.println(" deg");

  delay(50);
}
#include <Wire.h>

const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

const int MPU = 0x68;
float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

const int BASE_SPEED = 75;

void setup() {
  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF,  OUTPUT);
  pinMode(LeftB,  OUTPUT);

  Wire.begin();

  // Wake MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Calibrate — keep robot still for 4 seconds!
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    gyroZ_offset += (Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset /= 2000.0;

  previousTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);

  int16_t GyZ_raw = Wire.read() << 8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;
  if (abs(GyZ) < 0.5) GyZ = 0;
  yaw += GyZ * dt;

  // Clamp yaw so it can never cause spinning
  yaw = constrain(yaw, -10.0, 10.0);

  float correction = 0;
  if (abs(yaw) > 1.0) {
    correction = yaw * 1.5;
    // Clamp correction to max 20 — prevents spinning
    correction = constrain(correction, -20, 20);
  }

  int speedL = BASE_SPEED - (int)correction;
  int speedR = BASE_SPEED + (int)correction;

  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  analogWrite(LeftF,  speedL); digitalWrite(LeftB,  LOW);
  analogWrite(RightF, speedR); digitalWrite(RightB, LOW);
}
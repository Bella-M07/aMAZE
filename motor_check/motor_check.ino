#include <Wire.h>

// Motor Pins
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

const int MPU_ADDR = 0x68;

float angleZ = 0;
float gyroZ_offset = 0;
unsigned long previousTime;

float targetAngle = 90;
float Kp = 3.0;

int16_t gyroZ_raw;
float gyroZ;

enum State {MOVE_FORWARD, TURN_90, STOPPED};
State robotState = MOVE_FORWARD;

unsigned long moveStartTime;

void setMotor(int rightSpeed, int leftSpeed) {
  // Right Motor
  if (rightSpeed > 0) {
    analogWrite(RightF, rightSpeed);
    analogWrite(RightB, 0);
  } else {
    analogWrite(RightF, 0);
    analogWrite(RightB, -rightSpeed);
  }

  // Left Motor
  if (leftSpeed > 0) {
    analogWrite(LeftF, leftSpeed);
    analogWrite(LeftB, 0);
  } else {
    analogWrite(LeftF, 0);
    analogWrite(LeftB, -leftSpeed);
  }
}

void calibrateGyro() {
  long sum = 0;
  Serial.println("Calibrating... Keep still.");
  delay(3000);

  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    gyroZ_raw = Wire.read() << 8 | Wire.read();
    sum += gyroZ_raw;
    delay(2);
  }

  gyroZ_offset = (sum / 2000.0) / 131.0;
  Serial.println("Calibration done.");
}

void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  gyroZ_raw = Wire.read() << 8 | Wire.read();
  gyroZ = (gyroZ_raw / 131.0) - gyroZ_offset;

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  angleZ += gyroZ * dt;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();

  previousTime = millis();
  moveStartTime = millis();

  Serial.println("Moving Forward...");
}

void loop() {

  readGyro();

  switch (robotState) {

    case MOVE_FORWARD:
      setMotor(120, 120);

      if (millis() - moveStartTime > 2000) {  // Move 2 seconds
        setMotor(0, 0);
        delay(500);
        angleZ = 0;  // Reset angle before turn
        previousTime = millis();
        robotState = TURN_90;
        Serial.println("Starting 90 Degree Turn...");
      }
      break;

    case TURN_90: {
      float error = targetAngle - angleZ;
      float motorSpeed = Kp * error;
      motorSpeed = constrain(motorSpeed, -150, 150);

      setMotor(-motorSpeed, motorSpeed);

      Serial.print("Angle: ");
      Serial.print(angleZ);
      Serial.print("  Error: ");
      Serial.println(error);

      if (abs(error) < 1.0) {
        setMotor(0, 0);
        robotState = STOPPED;
        Serial.println("Turn Complete!");
      }
      break;
    }

    case STOPPED:
      setMotor(0, 0);
      break;
  }
}
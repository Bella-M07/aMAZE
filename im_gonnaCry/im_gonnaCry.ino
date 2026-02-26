#include <Wire.h>

// ---------------- PINS ----------------
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

const int trigPin = 7;
const int echoPin = 8;

const int MPU = 0x68;

// ---------------- MOTION SETTINGS ----------------
const int DRIVE_SPEED = 80;
const int MAX_TURN_SPEED = 180;

const float KP_STRAIGHT = 1.8;
const float KP_TURN = 3.0;   // Slightly reduced to prevent overshoot

// ---------------- GYRO ----------------
float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

// ---------------- HEADING SYSTEM ----------------
float targetHeading = 0;

enum State { MOVE_FORWARD, TURN_90 };
State robotState = MOVE_FORWARD;

// =======================================================
// ====================== SETUP ==========================
// =======================================================

void setup() {

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF,  OUTPUT);
  pinMode(LeftB,  OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();

  previousTime = millis();
}

// =======================================================
// ======================= LOOP ==========================
// =======================================================

void loop() {

  readGyro();

  if (robotState == MOVE_FORWARD) {

    if (getDistance() < 15) {

      fullStop();
      delay(400);

      targetHeading += 90.0;
      targetHeading = normalizeAngle(targetHeading);

      robotState = TURN_90;
    }
    else {
      moveStraight();
    }
  }

  else if (robotState == TURN_90) {
    performTurn();
  }
}

// =======================================================
// =================== MOVEMENT ==========================
// =======================================================

void moveStraight() {

  float error = normalizeAngle(targetHeading - yaw);

  float correction = error * KP_STRAIGHT;
  correction = constrain(correction, -40, 40);

  int speedL = DRIVE_SPEED - correction;
  int speedR = DRIVE_SPEED + correction;

  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  motors(speedL, speedR);
}

void performTurn() {

  float error = normalizeAngle(targetHeading - yaw);

  if (abs(error) > 2.0) {

    float turnPower = error * KP_TURN;
    turnPower = constrain(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);

    motors(turnPower, -turnPower);
  }
  else {
    fullStop();
    delay(500);
    robotState = MOVE_FORWARD;
  }
}

void motors(int left, int right) {

  if (left >= 0) {
    analogWrite(LeftF, left);
    analogWrite(LeftB, 0);
  } else {
    analogWrite(LeftF, 0);
    analogWrite(LeftB, abs(left));
  }

  if (right >= 0) {
    analogWrite(RightF, right);
    analogWrite(RightB, 0);
  } else {
    analogWrite(RightF, 0);
    analogWrite(RightB, abs(right));
  }
}

void fullStop() {
  digitalWrite(LeftF, LOW);
  digitalWrite(LeftB, LOW);
  digitalWrite(RightF, LOW);
  digitalWrite(RightB, LOW);
}

// =======================================================
// ==================== GYRO =============================
// =======================================================

void readGyro() {

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);

  int16_t GyZ_raw = Wire.read() << 8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;

  if (abs(GyZ) < 0.3) GyZ = 0;

  yaw += GyZ * dt;
  yaw = normalizeAngle(yaw);
}

// =======================================================
// ================== ULTRASONIC =========================
// =======================================================

long getDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 15000);

  if (duration == 0) return 999;

  return duration * 0.034 / 2;
}

// =======================================================
// ================= CALIBRATION =========================
// =======================================================

void calibrateGyro() {

  for (int i = 0; i < 2000; i++) {

    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);

    gyroZ_offset += (Wire.read() << 8 | Wire.read());
    delay(2);
  }

  gyroZ_offset /= 2000.0;
}

// =======================================================
// ================= ANGLE NORMALIZER ====================
// =======================================================

float normalizeAngle(float angle) {

  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;

  return angle;
}
#include <Wire.h>

// ---------------- MOTOR PINS ----------------
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// ---------------- ULTRASONIC ----------------
const int trigPin = 7;
const int echoPin = 5;

// ---------------- MPU ----------------
const int MPU_ADDR = 0x68;

float angleZ = 0;
float gyroZ_offset = 0;
unsigned long previousTime;

float targetAngle = 90;
float Kp = 1.5;     // LOWERED to prevent overshoot

int16_t gyroZ_raw;
float gyroZ;

enum State {MOVE_FORWARD, TURN_90};
State robotState = MOVE_FORWARD;

// ---------------- BASIC MOTOR (NO BOOST) ----------------
void setMotorBasic(int rightSpeed, int leftSpeed) {

  if (rightSpeed > 0) {
    analogWrite(RightF, rightSpeed);
    analogWrite(RightB, 0);
  } else {
    analogWrite(RightF, 0);
    analogWrite(RightB, abs(rightSpeed));
  }

  if (leftSpeed > 0) {
    analogWrite(LeftF, leftSpeed);
    analogWrite(LeftB, 0);
  } else {
    analogWrite(LeftF, 0);
    analogWrite(LeftB, abs(leftSpeed));
  }
}

// ---------------- MOTOR WITH START BOOST (FOR FORWARD ONLY) ----------------
void setMotorSmooth(int rightSpeed, int leftSpeed) {

  static bool rightStopped = true;
  static bool leftStopped  = true;

  int boostPWM = 120;
  int boostTime = 100;

  // RIGHT
  if (rightSpeed != 0 && rightStopped) {
    if (rightSpeed > 0) {
      analogWrite(RightF, boostPWM);
      analogWrite(RightB, 0);
    } else {
      analogWrite(RightF, 0);
      analogWrite(RightB, boostPWM);
    }
    delay(boostTime);
    rightStopped = false;
  }
  if (rightSpeed == 0) rightStopped = true;

  if (rightSpeed > 0) {
    analogWrite(RightF, abs(rightSpeed));
    analogWrite(RightB, 0);
  } else {
    analogWrite(RightF, 0);
    analogWrite(RightB, abs(rightSpeed));
  }

  // LEFT
  if (leftSpeed != 0 && leftStopped) {
    if (leftSpeed > 0) {
      analogWrite(LeftF, boostPWM);
      analogWrite(LeftB, 0);
    } else {
      analogWrite(LeftF, 0);
      analogWrite(LeftB, boostPWM);
    }
    delay(boostTime);
    leftStopped = false;
  }
  if (leftSpeed == 0) leftStopped = true;

  if (leftSpeed > 0) {
    analogWrite(LeftF, abs(leftSpeed));
    analogWrite(LeftB, 0);
  } else {
    analogWrite(LeftF, 0);
    analogWrite(LeftB, abs(leftSpeed));
  }
}

// ---------------- GYRO CALIBRATION ----------------
void calibrateGyro() {
  long sum = 0;

  Serial.println("Calibrating... KEEP STILL");
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

  Serial.print("Gyro Offset: ");
  Serial.println(gyroZ_offset);
}

// ---------------- READ GYRO ----------------
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

// ---------------- READ DISTANCE ----------------
long readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;

  return distance;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();
  previousTime = millis();

  Serial.println("Robot Started");
}

// ---------------- LOOP ----------------
void loop() {

  readGyro();
  long distance = readDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  switch (robotState) {

    case MOVE_FORWARD:

      if (distance < 20) {
        Serial.println("Obstacle Detected!");
        setMotorBasic(0, 0);
        delay(300);

        angleZ = 0;
        previousTime = millis();
        robotState = TURN_90;
      } else {
        setMotorSmooth(75, 75);
      }

      break;

    case TURN_90: {

      float error = targetAngle - angleZ;
      float motorSpeed = Kp * error;

      motorSpeed = constrain(motorSpeed, -120, 120);

      // Minimum turning power (no boost, no dead zone)
      if (abs(motorSpeed) < 70 && abs(error) > 2)
        motorSpeed = (motorSpeed > 0) ? 70 : -70;

      Serial.print("Turning | Angle: ");
      Serial.print(angleZ);
      Serial.print(" | Error: ");
      Serial.println(error);

      setMotorBasic(-motorSpeed, motorSpeed);

      if (abs(error) < 1.0) {
        Serial.println("Turn Complete");
        setMotorBasic(0, 0);
        delay(300);
        robotState = MOVE_FORWARD;
      }

      break;
    }
  }
}
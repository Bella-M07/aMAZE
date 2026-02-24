#include <Wire.h>

// -------- Motor Pins --------
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// -------- Ultrasonic --------
const int trigPin = 7;
const int echoPin = 5;

// -------- MPU --------
const int MPU = 0x68;
float gyroZ_offset = 0;
float yaw = 0;

unsigned long previousTime;
float dt;

long duration;
float distance;

const int MAX_SPEED = 90;
const int STOP_DISTANCE = 20;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Wake MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Calibrating MPU... Keep robot still!");

  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    gyroZ_offset += (Wire.read() << 8 | Wire.read());
    delay(2);
  }

  gyroZ_offset /= 2000.0;

  Serial.println("Calibration Complete!");
  Serial.println("Robot Ready.");
  previousTime = millis();
}

void loop() {

  distance = getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > STOP_DISTANCE) {
    Serial.println("Action: Moving Forward (MAX)");
    moveForward(MAX_SPEED);
  }
  else {
    Serial.println("Action: Obstacle Detected");
    stopMotors();
    delay(300);

    turnRight90();   // PRIORITY ACTION
  }

  Serial.println("----------------------------");
  delay(50);
}

// ================= FUNCTIONS =================

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void updateYaw() {

  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);

  int16_t GyZ_raw = Wire.read() << 8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;

  if (abs(GyZ) < 0.5) GyZ = 0;

  yaw += GyZ * dt;
}

void turnRight90() {

  yaw = 0;
  previousTime = millis();

  Serial.println("Action: Turning Right to 90°");

  while (true) {

    updateYaw();

    float error = 90 - yaw;

    // LOCK CONDITION
    if (abs(error) <= 1.0) {
      stopMotors();
      Serial.println("Action: 90° Locked");
      break;
    }

    // Smooth speed control
    if (error > 20) {
      turnRight(200);
    }
    else if (error > 5) {
      turnRight(120);
    }
    else {
      turnRight(60);
    }

    Serial.print("Yaw: ");
    Serial.println(yaw);
  }

  stopMotors();
  delay(300);
}

void moveForward(int speed) {
  analogWrite(RightF, speed);
  digitalWrite(RightB, LOW);

  analogWrite(LeftF, speed);
  digitalWrite(LeftB, LOW);
}

void turnRight(int speed) {
  analogWrite(RightF, speed);
  digitalWrite(RightB, LOW);

  digitalWrite(LeftF, LOW);
  analogWrite(LeftB, speed);
}

void stopMotors() {
  digitalWrite(RightF, LOW);
  digitalWrite(RightB, LOW);
  digitalWrite(LeftF, LOW);
  digitalWrite(LeftB, LOW);
}
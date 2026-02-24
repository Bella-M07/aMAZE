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

const int BASE_SPEED    = 90;
const int STOP_DISTANCE = 20;

// ================= SETUP =================

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF,  OUTPUT);
  pinMode(LeftB,  OUTPUT);

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

  // Kickstart — helps powerbank stay on
  analogWrite(RightF, 255);
  analogWrite(LeftF,  255);
  delay(100);

  previousTime = millis();
}

// ================= LOOP =================

void loop() {

  updateYaw();
  distance = getDistance();

  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  Serial.print("Yaw: ");      Serial.println(yaw);

  if (distance > 0 && distance <= STOP_DISTANCE) {
    Serial.println("Action: Obstacle Detected!");
    stopMotors();
    delay(300);

    // ------------------------------------------
    // Change this to turnLeft90() if you want
    // the robot to always turn left on obstacle
    // ------------------------------------------
    turnRight90();
  }
  else {
    // --- Straight line correction using yaw ---
    // If robot spirals the wrong way, swap += and -=
    float correction = yaw * 5.0;

    int speedL = BASE_SPEED + (int)correction;
    int speedR = BASE_SPEED - (int)correction;

    speedL = constrain(speedL, 0, 255);
    speedR = constrain(speedR, 0, 255);

    analogWrite(LeftF,  speedL); digitalWrite(LeftB,  LOW);
    analogWrite(RightF, speedR); digitalWrite(RightB, LOW);

    Serial.println("Action: Moving Forward (Corrected)");
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

  if (duration == 0) return 999;

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

    float error = 90.0 - yaw;

    if (abs(error) <= 1.5) break;

    int speed = constrain((int)(abs(error) * 2.5), 55, 200);
    turnRight(speed);

    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" | Error: "); Serial.println(error);
  }

  // Active Brake
  analogWrite(LeftF,  200); digitalWrite(LeftB,  LOW);
  analogWrite(RightB, 200); digitalWrite(RightF, LOW);
  delay(45);

  stopMotors();
  delay(400);

  yaw = 0;
  previousTime = millis();
  Serial.println("Action: Right 90° Locked");

  // Kickstart
  analogWrite(RightF, 255);
  analogWrite(LeftF,  255);
  delay(100);
}

void turnLeft90() {
  yaw = 0;
  previousTime = millis();

  Serial.println("Action: Turning Left to -90°");

  while (true) {
    updateYaw();

    // Left turn goes negative, so target is -90
    float error = -90.0 - yaw;

    if (abs(error) <= 1.5) break;

    int speed = constrain((int)(abs(error) * 2.5), 55, 200);
    turnLeft(speed);

    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" | Error: "); Serial.println(error);
  }

  // Active Brake — opposite of left turn motors
  analogWrite(RightF, 200); digitalWrite(RightB, LOW);
  analogWrite(LeftB,  200); digitalWrite(LeftF,  LOW);
  delay(45);

  stopMotors();
  delay(400);

  yaw = 0;
  previousTime = millis();
  Serial.println("Action: Left 90° Locked");

  // Kickstart
  analogWrite(RightF, 255);
  analogWrite(LeftF,  255);
  delay(100);
}

void turnRight(int speed) {
  analogWrite(RightF, speed); digitalWrite(RightB, LOW);
  digitalWrite(LeftF,  LOW);  analogWrite(LeftB,  speed);
}

void turnLeft(int speed) {
  analogWrite(LeftF,  speed); digitalWrite(LeftB,  LOW);
  digitalWrite(RightF, LOW);  analogWrite(RightB, speed);
}

void stopMotors() {
  digitalWrite(RightF, LOW);
  digitalWrite(RightB, LOW);
  digitalWrite(LeftF,  LOW);
  digitalWrite(LeftB,  LOW);
}
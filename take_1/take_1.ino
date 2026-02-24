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

const int BASE_SPEED   = 75;
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
  analogWrite(RightF, 75);
  analogWrite(LeftF, 75);
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
    turnRight90();
  }
  else {
    // --- Straight line correction using yaw ---
    // If robot spirals the wrong way, change += and -= below
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

  // If duration is 0, sensor timed out — ignore reading
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

    // Target reached
    if (abs(error) <= 1.5) {
      break;
    }

    // Proportional speed — naturally slows as it approaches target
    //int speed = constrain((int)(abs(error) * 2.5), 55, 200);
    turnRight();

    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" | Error: "); Serial.println(error);
  }

  // --- Active Brake --- counters momentum
  // Fires opposite motors briefly to kill overshoot
  analogWrite(LeftF,  75);
  analogWrite(RightB, 15);
  digitalWrite(LeftB,  LOW);
  digitalWrite(RightF, LOW);
  delay(45);  // Tune this: increase if overshoots, decrease if undershoots

  stopMotors();
  delay(400);

  // Reset yaw to 0 so straight-line correction works from new heading
  yaw = 0;
  previousTime = millis();

  Serial.println("Action: 90° Locked");

  // Kickstart forward after turn
  analogWrite(RightF, 75);
  analogWrite(LeftF,  75);
  delay(100);
}

void turnRight() {
  analogWrite(RightF, 100); digitalWrite(RightB, LOW);
  digitalWrite(LeftF,  LOW);  analogWrite(LeftB,  100);
}

void stopMotors() {
  digitalWrite(RightF, LOW);
  digitalWrite(RightB, LOW);
  digitalWrite(LeftF,  LOW);
  digitalWrite(LeftB,  LOW);
}
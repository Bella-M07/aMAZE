#include <Wire.h>

// ---------------- MOTOR PINS ----------------
const int RightF = 3, RightB = 9, LeftF = 10, LeftB = 11;

// ---------------- 3x ULTRASONIC PINS ----------------
const int trigL = A1; const int echoL = A0; // Left
const int trigC = 7;  const int echoC = 5;  // Center
const int trigR = A3; const int echoR = A2; // Right

// ---------------- MPU 6050 ----------------
const int MPU_ADDR = 0x68;
float angleZ = 0;
float gyroZ_offset = 0;
unsigned long previousTime;

// ---------------- TUNING ----------------
float targetAngle = 93.0; 
int turnPower = 120;      // Higher power for tight carpet turns
int driveSpeed = 90;      // Slightly slower for better sensor reaction
int torqueKick = 165;    

// Thresholds for a 27-29cm wide hallway
const int FRONT_WALL = 9;   // Stop 9cm from front wall
const int SIDE_OPEN = 18;   // If side sensor > 18cm, there is no wall there

enum State {MOVE_FORWARD, TURN_RIGHT, TURN_LEFT};
State robotState = MOVE_FORWARD;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT); pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT); pinMode(echoR, INPUT);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();
  previousTime = millis();
}

void loop() {
  readGyro(); 

  long distC = getDistance(trigC, echoC);
  long distR = getDistance(trigR, echoR);
  long distL = getDistance(trigL, echoL);

  switch (robotState) {
    case MOVE_FORWARD:
      // 1. Check if Right side is open (Priority #1 for Right Hand Rule)
      if (distR > SIDE_OPEN) {
        delay(100); // Small buffer to get slightly past the corner
        fullStop();
        angleZ = 0;
        robotState = TURN_RIGHT;
      } 
      // 2. Check if Front is blocked
      else if (distC < FRONT_WALL && distC > 0) {
        fullStop();
        angleZ = 0;
        // If front is blocked, check Left as the escape route
        if (distL > SIDE_OPEN) {
          robotState = TURN_LEFT;
        } else {
          // If Right and Front are blocked, Left is the only way out (U-turn)
          robotState = TURN_LEFT; 
        }
      } 
      else {
        motors(driveSpeed, driveSpeed);
      }
      break;

    case TURN_RIGHT:
      if (abs(angleZ) < targetAngle) {
        int power = (abs(angleZ) < 12) ? torqueKick : turnPower;
        motors(-power, power); 
      } else {
        finishTurn();
      }
      break;

    case TURN_LEFT:
      if (abs(angleZ) < targetAngle) {
        int power = (abs(angleZ) < 12) ? torqueKick : turnPower;
        motors(power, -power); 
      } else {
        finishTurn();
      }
      break;
  }
}

// ---------------- HELPERS ----------------

void finishTurn() {
  fullStop();
  delay(400); 
  angleZ = 0; 
  robotState = MOVE_FORWARD;
}

void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  
  if (Wire.available() >= 2) {
    int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();
    float gyroZ = (gyroZ_raw / 131.0) - gyroZ_offset;
    if (abs(gyroZ) < 0.2) gyroZ = 0; 

    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    angleZ += gyroZ * dt;
  }
}

long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 15000); 
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void motors(int left, int right) {
  if (left >= 0) { analogWrite(LeftF, left); analogWrite(LeftB, 0); }
  else { analogWrite(LeftF, 0); analogWrite(LeftB, abs(left)); }
  if (right >= 0) { analogWrite(RightF, right); analogWrite(RightB, 0); }
  else { analogWrite(RightF, 0); analogWrite(RightB, abs(right)); }
}

void fullStop() {
  digitalWrite(LeftF, HIGH); digitalWrite(LeftB, HIGH);
  digitalWrite(RightF, HIGH); digitalWrite(RightB, HIGH);
}

void calibrateGyro() {
  float sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = (sum / 500.0) / 131.0;
}
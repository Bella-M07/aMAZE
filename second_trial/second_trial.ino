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

// ---------------- TUNING (ADJUSTED FOR PRECISION) ----------------
float targetAngle = 93.0; // Increased to fix the "less than 90" issue
int turnPower = 115;      // More power to fight surface friction
int driveSpeed = 100;    
int torqueKick = 160;     // Strong burst to start the turn

enum State {MOVE_FORWARD, DECIDE, TURN_RIGHT, TURN_LEFT};
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
  readGyro(); // This must run every single loop

  switch (robotState) {
    case MOVE_FORWARD:
      if (getDistance(trigC, echoC) < 20) {
        fullStop();
        delay(400);
        robotState = DECIDE;
      } else {
        motors(driveSpeed, driveSpeed);
      }
      break;

    case DECIDE: {
      long distR = getDistance(trigR, echoR);
      long distL = getDistance(trigL, echoL);
      
      angleZ = 0; // Reset angle right before we start turning
      
      if (distR > 25) {
        robotState = TURN_RIGHT;
      } else if (distL > 25) {
        robotState = TURN_LEFT;
      } else {
        robotState = TURN_LEFT; // Default for U-turn/dead end
      }
      break;
    }

    case TURN_RIGHT: {
      float currentAngle = abs(angleZ);
      if (currentAngle < targetAngle) {
        int power = (currentAngle < 12) ? torqueKick : turnPower;
        motors(-power, power); 
      } else {
        finishTurn();
      }
      break;
    }

    case TURN_LEFT: {
      float currentAngle = abs(angleZ);
      if (currentAngle < targetAngle) {
        int power = (currentAngle < 12) ? torqueKick : turnPower;
        motors(power, -power); 
      } else {
        finishTurn();
      }
      break;
    }
  }
}

// ---------------- HELPERS ----------------

void finishTurn() {
  fullStop(); // Slam the brakes
  delay(500); // Wait for physical wobble to stop
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
    
    if (abs(gyroZ) < 0.2) gyroZ = 0; // Filter noise

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
  if (left >= 0) { 
    analogWrite(LeftF, left); analogWrite(LeftB, 0); 
  } else { 
    analogWrite(LeftF, 0); analogWrite(LeftB, abs(left)); 
  }
  
  if (right >= 0) { 
    analogWrite(RightF, right); analogWrite(RightB, 0); 
  } else { 
    analogWrite(RightF, 0); analogWrite(RightB, abs(right)); 
  }
}

void fullStop() {
  // Use HIGH/HIGH to create a magnetic brake effect
  digitalWrite(LeftF, HIGH); digitalWrite(LeftB, HIGH);
  digitalWrite(RightF, HIGH); digitalWrite(RightB, HIGH);
}

void calibrateGyro() {
  float sum = 0;
  Serial.println("Calibrating... Keep robot still!");
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = (sum / 500.0) / 131.0;
  Serial.println("Calibration Done.");
}
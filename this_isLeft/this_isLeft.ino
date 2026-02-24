#include <Wire.h>

// ---------------- PINS ----------------
const int RightF = 3, RightB = 9, LeftF = 10, LeftB = 11;
const int trigPin = 8, echoPin = 7; 
const int MPU_ADDR = 0x68;

float angleZ = 0;
float gyroZ_offset = 0;
unsigned long previousTime;

// --- TUNED VALUES FROM CODE 2 ---
float targetAngle = 91.0; // Over-compensated for friction
int turnPower = 115;      // Increased for torque
int driveSpeed = 100;     // Forward power
int torqueKick = 160;     // Initial burst to break static friction
// --------------------------------------

int16_t gyroZ_raw;
float gyroZ;
enum State {MOVE_FORWARD, TURN_LEFT};
State robotState = MOVE_FORWARD;

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();
  previousTime = millis();
  Serial.println("Robot Ready with Code 2 Updates!");
}

void loop() {
  readGyro(); // Must run every loop

  if (robotState == MOVE_FORWARD) {
    // Using Code 2's 9cm tight-sensing logic
    if (getDistance() < 9) {
      fullStop();
      delay(400);
      angleZ = 0; // Reset angle for a clean turn
      robotState = TURN_LEFT;
    } else {
      motors(driveSpeed, driveSpeed);
    }
  } 
  
  else if (robotState == TURN_LEFT) {
    float currentAbsAngle = abs(angleZ);
    
    if (currentAbsAngle < targetAngle) {
      // CODE 2 UPDATE: Torque Kick for the first 12 degrees
      int power = (currentAbsAngle < 12) ? torqueKick : turnPower;
      
      // Pivot Left: Right wheels Forward, Left wheels Backward
      motors(-power, power); 
    } else {
      fullStop();
      Serial.print("Left Turn Done! Final Angle: "); Serial.println(angleZ);
      delay(800); 
      robotState = MOVE_FORWARD;
    }
  }
}

// ---------------- CORE FUNCTIONS ----------------

void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  
  if (Wire.available() >= 2) {
    gyroZ_raw = Wire.read() << 8 | Wire.read();
    gyroZ = (gyroZ_raw / 131.0) - gyroZ_offset;
    
    if (abs(gyroZ) < 0.2) gyroZ = 0; // Deadzone filter

    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    angleZ += gyroZ * dt;
  }
}

void motors(int left, int right) {
  if (left >= 0) { analogWrite(LeftF, left); analogWrite(LeftB, 0); }
  else { analogWrite(LeftF, 0); analogWrite(LeftB, abs(left)); }
  
  if (right >= 0) { analogWrite(RightF, right); analogWrite(RightB, 0); }
  else { analogWrite(RightF, 0); analogWrite(RightB, abs(right)); }
}

void fullStop() {
  // Active braking by pulling all pins HIGH
  digitalWrite(LeftF, HIGH); digitalWrite(LeftB, HIGH);
  digitalWrite(RightF, HIGH); digitalWrite(RightB, HIGH);
}

long getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 15000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

void calibrateGyro() {
  float sum = 0;
  Serial.println("Calibrating... DO NOT MOVE");
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = (sum / 500.0) / 131.0;
  Serial.println("Calibration finished.");
}
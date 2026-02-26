#include <Wire.h>

// ---------------- PINS ----------------
const int RightF = 3, RightB = 9, LeftF = 10, LeftB = 11;
const int trigPin = 7, echoPin = 8; 
const int MPU_ADDR = 0x68;

// ---------------- CONSTANTS ----------------
const int BASE_SPEED = 80;       // Speed for walking straight
const int MAX_TURN_SPEED = 160;  
const int MIN_TURN_SPEED = 100;  
const int TORQUE_KICK = 255;     // High power to break friction
const float RAMP_DN_DEG = 35.0;  // Degrees remaining to start slowing down

// ---------------- VARIABLES ----------------
float yaw = 0;
float gyroZ_offset = 0;
unsigned long previousTime;
unsigned long moveStartTime = 0; 

enum State {MOVE_FORWARD, TURN_90};
State robotState = MOVE_FORWARD;

void setup() {
  Serial.begin(115200); 
  Wire.begin();
  
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  // Wake MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();
  previousTime = millis();
}

void loop() {
  updateYaw(); 

  if (robotState == MOVE_FORWARD) {
    if (getDistance() < 15) { 
      fullStop();
      delay(500);
      yaw = 0;            // Reset yaw before the turn
      robotState = TURN_90;
    } else {
      straight();         // Logic from Code 1
    }
  } 
  else if (robotState == TURN_90) {
    turnRight(90.0);      // Logic from Code 2
  }
}

// ---------------- NAVIGATION FUNCTIONS ----------------

// Based on Code 1: Uses yaw to keep the robot driving in a straight line
void straight() {
  float correction = 0;
  
  // If we drift more than 1 degree, apply correction
  if (abs(yaw) > 1.0) {
    correction = yaw * 1.5;
    correction = constrain(correction, -20, 20); // Prevent wild spinning
  }

  int speedL = BASE_SPEED - (int)correction;
  int speedR = BASE_SPEED + (int)correction;

  motors(speedL, speedR);
}

// Based on Code 2: Smoothly rotates the robot to a specific angle
void turnRight(float targetAngle) {
  float currentAbsAngle = abs(yaw);
  float remainingAngle = targetAngle - currentAbsAngle;

  if (remainingAngle > 0.5) { 
    int power;
    
    if (currentAbsAngle < 12) {
      power = TORQUE_KICK; // Initial shove to start turning
    } 
    else if (remainingAngle < RAMP_DN_DEG) {
      power = map(remainingAngle, 0, RAMP_DN_DEG, MIN_TURN_SPEED, MAX_TURN_SPEED);
    } 
    else {
      power = MAX_TURN_SPEED;
    }

    power = constrain(power, MIN_TURN_SPEED, TORQUE_KICK);
    motors(power, -power); // Pivot turn: Left forward, Right back
  } 
  else {
    fullStop();
    delay(800); 
    yaw = 0;               // Reset yaw so 'straight()' starts at 0
    robotState = MOVE_FORWARD;
  }
}

// ---------------- CORE UTILITIES ----------------

void updateYaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  
  if (Wire.available() >= 2) {
    int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();
    float gyroZ = (gyroZ_raw / 131.0) - gyroZ_offset;
    
    if (abs(gyroZ) < 0.15) gyroZ = 0; // Deadzone to stop drift

    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;
    
    // Protection against huge dt jumps
    if (dt > 0.1) dt = 0.01; 
    
    previousTime = currentTime;
    yaw += gyroZ * dt;
  }
}

void motors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0) { analogWrite(LeftF, left); analogWrite(LeftB, 0); }
  else { analogWrite(LeftF, 0); analogWrite(LeftB, abs(left)); }
  
  if (right >= 0) { analogWrite(RightF, right); analogWrite(RightB, 0); }
  else { analogWrite(RightF, 0); analogWrite(RightB, abs(right)); }
}

void fullStop() {
  digitalWrite(LeftF, 0); digitalWrite(LeftB, 0);
  digitalWrite(RightF, 0); digitalWrite(RightB, 0);
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
  int samples = 2000;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = (sum / (float)samples) / 131.0;
}
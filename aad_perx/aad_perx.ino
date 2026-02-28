#include <Wire.h>

// ================= MOTOR PINS =================
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// ================= FRONT ULTRASONIC =================
const int trigPin = 7;
const int echoPin = 8;

// ================= RIGHT ULTRASONIC =================
const int trigRight = A3;
const int echoRight = A2;

// ================= LEFT ULTRASONIC =================
const int trigLeft = A1;
const int echoLeft = A0;

// ================= MPU =================
const int MPU = 0x68;

// ================= LED =================
const int rightLedPin = 5;

// ================= GYRO =================
float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

// ================= DRIVE =================
const int BASE_SPEED = 110;
const int SLOW_SPEED = 70;
const float KP = 10;
const int MAX_CORRECTION = 80;

// ================= DISTANCE =================
const int SAFE_DISTANCE = 25;  // Increased for safety
const int WALL_DISTANCE = 25;  // Threshold for "open" path

// ================= TURN =================
const float TARGET_TURN_90  = 75.0;
const float TARGET_TURN_180 = 175.0;
const int TURN_SPEED = 200;

// ================= STATE TIMING =================
unsigned long stateStartTime = 0;
const unsigned long JUNCTION_LOCK_TIME = 800;  // ms to ignore junction after turn
const unsigned long FORWARD_MIN_TIME = 600;    // ms minimum forward after turn

// ================= FLAGS =================
bool rightTurnSequence = false;
bool leftTurnSequence  = false;
bool deadEndTurn       = false;
bool junctionLocked    = false;  // Prevent re-checking same junction

// ================= STATES =================
enum RobotState {
  STATE_NORMAL,
  STATE_STOPPED,
  STATE_TURNING_RIGHT,
  STATE_TURNING_LEFT,
  STATE_TURNING_180,
  STATE_FORWARD_LOCK
};

RobotState currentState = STATE_NORMAL;

// ================= SMOOTHED DISTANCE =================
float distances[3];  // front, right, left

float getDistanceCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return 999;

  return duration * 0.034 / 2;
}

float getDistanceCMSmooth(int trig, int echo) {
  float readings[5];
  for (int i = 0; i < 5; i++) {
    readings[i] = getDistanceCM(trig, echo);
    delay(2);
  }
  
  // Simple median filter for noise reduction
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (readings[i] > readings[j]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  return readings[2];  // Median
}

// ================= SETUP =================
void setup() {
  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(rightLedPin, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Gyro calibration (unchanged)
  long sum = 0;
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = sum / 2000.0;
  previousTime = millis();
}

// ================= DRIVE FUNCTIONS =================
void stopMotors() {
  analogWrite(LeftF, 0);
  analogWrite(LeftB, 0);
  analogWrite(RightF, 0);
  analogWrite(RightB, 0);
}

void driveStraight(float correction) {
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  
  analogWrite(LeftF, BASE_SPEED + correction);
  digitalWrite(LeftB, LOW);
  analogWrite(RightF, BASE_SPEED - correction);
  digitalWrite(RightB, LOW);
}

void turnRight(int speed) {
  digitalWrite(LeftB, LOW);
  digitalWrite(RightF, LOW);
  analogWrite(LeftF, speed);
  analogWrite(RightB, speed);
}

void turnLeft(int speed) {
  digitalWrite(LeftF, LOW);
  digitalWrite(RightB, LOW);
  analogWrite(LeftB, speed);
  analogWrite(RightF, speed);
}

// ================= LOOP =================
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // ===== READ GYRO =====
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);

  int16_t GyZ_raw = Wire.read() << 8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;

  if (abs(GyZ) < 0.1) GyZ = 0;
  yaw += GyZ * dt;

  switch (currentState) {

    // ================= NORMAL (Right Hand Rule) =================
    case STATE_NORMAL: {
      // Read smoothed distances
      distances[0] = getDistanceCMSmooth(trigPin, echoPin);   // front
      distances[1] = getDistanceCMSmooth(trigRight, echoRight); // right
      distances[2] = getDistanceCMSmooth(trigLeft, echoLeft);   // left

      bool rightOpen = distances[1] > WALL_DISTANCE;
      bool frontOpen = distances[0] > SAFE_DISTANCE;
      bool leftOpen = distances[2] > WALL_DISTANCE;

      // Junction lock: don't re-check for 800ms after turn
      if (junctionLocked && (millis() - stateStartTime < JUNCTION_LOCK_TIME)) {
        driveStraight(yaw * KP);
        break;
      }

      // 1️⃣ RIGHT PRIORITY
      if (rightOpen) {
        digitalWrite(rightLedPin, HIGH);  // LED on for right turn
        rightTurnSequence = true;
        leftTurnSequence = false;
        deadEndTurn = false;
        junctionLocked = true;
        currentState = STATE_STOPPED;
        break;
      }

      // 2️⃣ STRAIGHT
      if (frontOpen) {
        driveStraight(yaw * KP);
        break;
      }

      // 3️⃣ LEFT
      if (leftOpen) {
        digitalWrite(rightLedPin, HIGH);  // LED on for left turn too
        rightTurnSequence = false;
        leftTurnSequence = true;
        deadEndTurn = false;
        junctionLocked = true;
        currentState = STATE_STOPPED;
        break;
      }

      // 4️⃣ DEAD END
      rightTurnSequence = false;
      leftTurnSequence = false;
      deadEndTurn = true;
      junctionLocked = true;
      currentState = STATE_STOPPED;
    }
    break;

    // ================= STOP =================
    case STATE_STOPPED:
      stopMotors();
      yaw = 0;  // Reset yaw before turning

      if (deadEndTurn)
        currentState = STATE_TURNING_180;
      else if (rightTurnSequence)
        currentState = STATE_TURNING_RIGHT;
      else if (leftTurnSequence)
        currentState = STATE_TURNING_LEFT;
      else
        currentState = STATE_NORMAL;
      stateStartTime = millis();
    break;

    // ================= RIGHT TURN =================
    case STATE_TURNING_RIGHT:
      if (abs(yaw) >= TARGET_TURN_90) {
        stopMotors();
        stateStartTime = millis();
        currentState = STATE_FORWARD_LOCK;
        break;
      }
      turnRight(TURN_SPEED);
    break;

    // ================= LEFT TURN =================
    case STATE_TURNING_LEFT:
      if (abs(yaw) >= TARGET_TURN_90) {
        stopMotors();
        stateStartTime = millis();
        currentState = STATE_FORWARD_LOCK;
        break;
      }
      turnLeft(TURN_SPEED);
    break;

    // ================= 180 TURN =================
    case STATE_TURNING_180:
      if (abs(yaw) >= TARGET_TURN_180) {
        stopMotors();
        deadEndTurn = false;
        yaw = 0;
        currentState = STATE_NORMAL;
        break;
      }
      turnRight(TURN_SPEED);  // Right turn for 180 (consistent with right-hand rule)
    break;

    // ================= FORWARD LOCK =================
    case STATE_FORWARD_LOCK: {
      float frontDist = getDistanceCM(trigPin, echoPin);
      int moveSpeed = (frontDist < 30) ? SLOW_SPEED : BASE_SPEED;

      driveStraight(0);  // No yaw correction during lock to avoid drift

      // Exit after minimum time + front clearance check
      if ((millis() - stateStartTime >= FORWARD_MIN_TIME) && (frontDist > SAFE_DISTANCE + 5)) {
        digitalWrite(rightLedPin, LOW);
        rightTurnSequence = false;
        leftTurnSequence = false;
        junctionLocked = false;  // Unlock junction checking
        yaw = 0;
        currentState = STATE_NORMAL;
      }
    }
    break;
  }
}

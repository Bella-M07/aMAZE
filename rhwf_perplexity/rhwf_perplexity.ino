#include <Wire.h>

// ================= MOTOR PINS =================
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// ================= ULTRASONICS =================
const int trigFront = 7;
const int echoFront = 8;
const int trigLeft  = 5;
const int echoLeft  = 6;
const int trigRight = 4;
const int echoRight = 2;

// ================= MPU =================
const int MPU = 0x68;

float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

// ================= DRIVE PARAMETERS =================
const int BASE_SPEED = 150;
const float KP = 10;
const int MIN_KICK = 25;
const int MAX_CORRECTION = 80;

// ================= WALL FOLLOWING =================
const int WALL_DISTANCE = 25;      // Target right wall distance
const int WALL_TOO_CLOSE = 15;     // Too close to wall - turn away
const int WALL_TOO_FAR   = 40;     // Too far from wall - turn toward

// ================= OBSTACLE =================
const int FRONT_SAFE = 35;
const int FRONT_SLOW  = 45;

// ================= TURN PARAMETERS =================
const float TARGET_TURN = 87.0;
const int TURN_MIN_SPEED = 200;
const int TURN_MAX_SPEED = 230;
float turnSpeedFactor = 0;
unsigned long stateStartTime = 0;

// ================= STATE MACHINE =================
enum RobotState {
  STATE_FOLLOW_WALL,
  STATE_SLOWING_FRONT,
  STATE_STOPPED,
  STATE_WAIT_BEFORE_TURN,
  STATE_TURNING_RIGHT,
  STATE_TURNING_LEFT,
  STATE_TURN_FINISHED,
  STATE_WAIT_AFTER_TURN,
  STATE_RECOVER_WALL
};

RobotState currentState = STATE_FOLLOW_WALL;
float slowdownFactor = 1.0;

// ================= DISTANCE FUNCTIONS =================
float getDistanceFront() {
  digitalWrite(trigFront, LOW); delayMicroseconds(2);
  digitalWrite(trigFront, HIGH); delayMicroseconds(10);
  digitalWrite(trigFront, LOW);
  long duration = pulseIn(echoFront, HIGH, 30000);
  if(duration == 0) return 999;
  return duration * 0.034 / 2;
}

float getDistanceLeft() {
  digitalWrite(trigLeft, LOW); delayMicroseconds(2);
  digitalWrite(trigLeft, HIGH); delayMicroseconds(10);
  digitalWrite(trigLeft, LOW);
  long duration = pulseIn(echoLeft, HIGH, 30000);
  if(duration == 0) return 999;
  return duration * 0.034 / 2;
}

float getDistanceRight() {
  digitalWrite(trigRight, LOW); delayMicroseconds(2);
  digitalWrite(trigRight, HIGH); delayMicroseconds(10);
  digitalWrite(trigRight, LOW);
  long duration = pulseIn(echoRight, HIGH, 30000);
  if(duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);

  // Ultrasonic pins
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  Wire.begin();
  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  Serial.println("CALIBRATING MPU");
  long sum = 0;
  for(int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = sum / 2000.0;
  Serial.println("READY");
  previousTime = millis();
}

// ================= LOOP =================
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Read gyro
  Wire.beginTransmission(MPU); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  int16_t GyZ_raw = Wire.read() << 8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;
  if(abs(GyZ) < 0.1) GyZ = 0;
  yaw += GyZ * dt;

  // Read distances
  float distFront = getDistanceFront();
  float distLeft  = getDistanceLeft();
  float distRight = getDistanceRight();

  // ================= STATE MACHINE =================
  switch(currentState) {
    case STATE_FOLLOW_WALL:
      slowdownFactor = 1.0;
      
      // Front obstacle check
      if(distFront <= FRONT_SLOW) {
        currentState = STATE_SLOWING_FRONT;
      }
      // Wall following adjustments handled in motor control
      break;

    case STATE_SLOWING_FRONT:
      slowdownFactor -= 0.05;
      if(distFront <= FRONT_SAFE) slowdownFactor -= 0.1;
      slowdownFactor = constrain(slowdownFactor, 0, 1);
      
      if(slowdownFactor <= 0.05 || distFront <= 20) {
        slowdownFactor = 0;
        currentState = STATE_STOPPED;
        stateStartTime = millis();
      }
      break;

    case STATE_STOPPED:
      slowdownFactor = 0;
      currentState = STATE_WAIT_BEFORE_TURN;
      stateStartTime = millis();
      break;

    case STATE_WAIT_BEFORE_TURN:
      if(millis() - stateStartTime > 500) {
        yaw = 0;
        turnSpeedFactor = 0;
        // Decide turn direction based on right wall follower logic
        if(distRight < WALL_TOO_CLOSE && distLeft > WALL_TOO_FAR) {
          currentState = STATE_TURNING_RIGHT;  // Turn toward right wall
        } else {
          currentState = STATE_TURNING_LEFT;   // Obstacle ahead, turn left
        }
      }
      break;

    case STATE_TURNING_RIGHT:
    case STATE_TURNING_LEFT: {
      float remaining = TARGET_TURN - abs(yaw);
      if(remaining <= 0) {
        currentState = STATE_TURN_FINISHED;
        stateStartTime = millis();
        break;
      }
      
      if(abs(yaw) < 30) turnSpeedFactor += 0.02;
      if(remaining < 30) turnSpeedFactor -= 0.04;
      turnSpeedFactor = constrain(turnSpeedFactor, 0.4, 1.0);
    }
    break;

    case STATE_TURN_FINISHED:
      stopMotors();
      currentState = STATE_WAIT_AFTER_TURN;
      stateStartTime = millis();
      break;

    case STATE_WAIT_AFTER_TURN:
      if(millis() - stateStartTime > 500) {
        yaw = 0;
        currentState = STATE_RECOVER_WALL;
      }
      break;

    case STATE_RECOVER_WALL:
      if(distRight > WALL_TOO_FAR || distFront > FRONT_SAFE) {
        currentState = STATE_FOLLOW_WALL;
      }
      break;
  }

  // ================= MOTOR CONTROL =================
  if(currentState == STATE_TURNING_LEFT) {
    int turnSpeed = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * turnSpeedFactor;
    // Left turn: Left backward, Right forward
    digitalWrite(LeftF, LOW);  analogWrite(LeftB, turnSpeed);
    analogWrite(RightF, turnSpeed); digitalWrite(RightB, LOW);
  }
  else if(currentState == STATE_TURNING_RIGHT) {
    int turnSpeed = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * turnSpeedFactor;
    // Right turn: Left forward, Right backward
    analogWrite(LeftF, turnSpeed); digitalWrite(LeftB, LOW);
    digitalWrite(RightF, LOW); analogWrite(RightB, turnSpeed);
  }
  else if(currentState == STATE_FOLLOW_WALL || 
          currentState == STATE_SLOWING_FRONT || 
          currentState == STATE_RECOVER_WALL) {
    
    // WALL FOLLOWING + STRAIGHTENING LOGIC
    float wallCorrection = 0;
    float straightCorrection = 0;
    
    // 1. Wall following correction (right hand rule)
    if(distRight < WALL_TOO_CLOSE) {
      wallCorrection = -15;  // Turn away from wall
    } else if(distRight > WALL_TOO_FAR) {
      wallCorrection = 15;   // Turn toward wall
    }
    
    // 2. Straightening correction (gyro)
    if(abs(yaw) > 0.3) {
      straightCorrection = yaw * KP;
      if(straightCorrection > 0) straightCorrection += MIN_KICK;
      if(straightCorrection < 0) straightCorrection -= MIN_KICK;
      straightCorrection = constrain(straightCorrection, -MAX_CORRECTION, MAX_CORRECTION);
    }
    
    float totalCorrection = wallCorrection + straightCorrection;
    totalCorrection = constrain(totalCorrection, -MAX_CORRECTION, MAX_CORRECTION);
    
    int speedL = BASE_SPEED + totalCorrection;
    int speedR = BASE_SPEED - totalCorrection;
    
    int finalSpeedL = speedL * slowdownFactor;
    int finalSpeedR = speedR * slowdownFactor;
    
    analogWrite(LeftF, finalSpeedL);  digitalWrite(LeftB, LOW);
    analogWrite(RightF, finalSpeedR); digitalWrite(RightB, LOW);
  }
}

void stopMotors() {
  analogWrite(LeftF, 0); analogWrite(RightF, 0);
  analogWrite(LeftB, 0); analogWrite(RightB, 0);
}

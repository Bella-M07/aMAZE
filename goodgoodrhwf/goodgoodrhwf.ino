#include <Wire.h>

// ================= MOTOR PINS =================
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// ================= ULTRASONIC =================
const int trigFront = 7;
const int echoFront = 8;
const int trigRight = A3;
const int echoRight = A2;
const int trigLeft  = A1;
const int echoLeft  = A0;

// ================= MPU6050 =================
const int MPU = 0x68;
const int rightLedPin = 5;

// ================= GYRO VARIABLES =================
float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

// ================= DRIVE SETTINGS =================
const int BASE_SPEED   = 120; // Adjusted for better torque
const int SLOW_SPEED   = 80;
const int TURN_SPEED   = 200; // Slightly lower for better gyro accuracy
const int MAX_CORRECTION = 90;
const int SAFE_DISTANCE  = 18; // CM to wall in front

// ================= PID GAINS =================
const float KP_CENTER = 7.0;  // Steering power based on wall distance
const float KD_CENTER = 3.0;  // Dampens oscillations
const float KP_YAW    = 2.5;  // Keeps the nose straight

float previousCenterError = 0;
float filteredLeft  = 15;
float filteredRight = 15;

// ================= TURN TARGETS =================
const float TARGET_TURN_90  = 78.0;  // Adjusted for momentum overshoot
const float TARGET_TURN_180 = 170.0;

unsigned long stateStartTime = 0;

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
bool rightTurnSequence = false;
bool leftTurnSequence  = false;
bool deadEndTurn       = false;

// ================= UTILITY FUNCTIONS =================

float getDistanceCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Timeout reduced to 15ms (~2.5 meters) for faster loop processing
  long duration = pulseIn(echo, HIGH, 15000); 
  if(duration == 0) return 100; // Cap distance to 100cm if no wall

  return duration * 0.034 / 2;
}

void stopMotors() {
  analogWrite(LeftF, 0);  analogWrite(RightF, 0);
  analogWrite(LeftB, 0);  analogWrite(RightB, 0);
}

// ================= SETUP =================

void setup() {
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(rightLedPin, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  // Gyro Calibration - ROBOT MUST BE STILL
  long sum = 0;
  for(int i=0; i<1500; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    sum += (int16_t)(Wire.read()<<8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = sum / 1500.0;
  previousTime = millis();
}

// ================= MAIN LOOP =================

void loop() {
  // Update Delta Time and Yaw
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  int16_t GyZ_raw = Wire.read()<<8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;
  if(abs(GyZ) < 0.15) GyZ = 0; // Deadzone
  yaw += GyZ * dt;

  switch(currentState) {

    case STATE_NORMAL: {
      float fDist = getDistanceCM(trigFront, echoFront);
      float rDist = getDistanceCM(trigRight, echoRight);
      float lDist = getDistanceCM(trigLeft, echoLeft);

      // Low Pass Filter for wall smoothing
      filteredLeft  = 0.6 * filteredLeft  + 0.4 * lDist;
      filteredRight = 0.6 * filteredRight + 0.4 * rDist;

      // Decision Logic (Right Hand Rule)
      if(rDist > 30) { // Gap on the right found
        rightTurnSequence = true;
        currentState = STATE_STOPPED;
        break;
      } 
      
      if(fDist > SAFE_DISTANCE) { // Drive Straight
        float centerError = 0;
        
        // Only use wall centering if BOTH walls are visible
        if(filteredLeft < 25 && filteredRight < 25) {
            centerError = filteredLeft - filteredRight;
        } 
        // If only one wall is visible, just use Gyro to stay straight
        else {
            centerError = 0; 
        }

        float derivative = centerError - previousCenterError;
        previousCenterError = centerError;

        float correction = (KP_CENTER * centerError) + (KD_CENTER * derivative) + (KP_YAW * yaw);
        correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

        analogWrite(LeftF,  constrain(BASE_SPEED - correction, 0, 255));
        analogWrite(RightF, constrain(BASE_SPEED + correction, 0, 255));
        digitalWrite(LeftB, LOW); digitalWrite(RightB, LOW);
      } 
      else if(lDist > 30) { // Front blocked, try Left
        leftTurnSequence = true;
        currentState = STATE_STOPPED;
      } 
      else { // All blocked, Dead End
        deadEndTurn = true;
        currentState = STATE_STOPPED;
      }
    } break;

    case STATE_STOPPED:
      stopMotors();
      delay(100); // Short settle time
      yaw = 0; 
      previousCenterError = 0;
      if(deadEndTurn)      currentState = STATE_TURNING_180;
      else if(rightTurnSequence) currentState = STATE_TURNING_RIGHT;
      else if(leftTurnSequence)  currentState = STATE_TURNING_LEFT;
      else                 currentState = STATE_NORMAL;
    break;

    case STATE_TURNING_RIGHT:
      if(abs(yaw) >= TARGET_TURN_90) {
        stopMotors();
        stateStartTime = millis();
        currentState = STATE_FORWARD_LOCK;
      } else {
        analogWrite(LeftF, TURN_SPEED); digitalWrite(LeftB, LOW);
        analogWrite(RightB, TURN_SPEED); digitalWrite(RightF, LOW);
      }
    break;

    case STATE_TURNING_LEFT:
      if(abs(yaw) >= TARGET_TURN_90) {
        stopMotors();
        stateStartTime = millis();
        currentState = STATE_FORWARD_LOCK;
      } else {
        analogWrite(LeftB, TURN_SPEED); digitalWrite(LeftF, LOW);
        analogWrite(RightF, TURN_SPEED); digitalWrite(RightB, LOW);
      }
    break;

    case STATE_TURNING_180:
      if(abs(yaw) >= TARGET_TURN_180) {
        stopMotors();
        deadEndTurn = false;
        yaw = 0;
        currentState = STATE_NORMAL;
      } else {
        analogWrite(LeftF, TURN_SPEED); digitalWrite(LeftB, LOW);
        analogWrite(RightB, TURN_SPEED); digitalWrite(RightF, LOW);
      }
    break;

    case STATE_FORWARD_LOCK: {
      // Move forward slightly after a turn to clear the junction
      analogWrite(LeftF, BASE_SPEED);  digitalWrite(LeftB, LOW);
      analogWrite(RightF, BASE_SPEED); digitalWrite(RightB, LOW);

      if(millis() - stateStartTime >= 600) { // Time to clear the turn
        rightTurnSequence = false;
        leftTurnSequence  = false;
        yaw = 0;
        currentState = STATE_NORMAL;
      }
    } break;
  }
}
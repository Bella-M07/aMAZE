#include <Wire.h>

// ================= MOTOR PINS =================
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// ================= ULTRASONICS =================
// FRONT
const int trigFront = 7;
const int echoFront = 8;

// RIGHT
const int trigRight = 4;
const int echoRight = 5;

// LEFT
const int trigLeft = 12;
const int echoLeft = 13;

// ================= MPU =================
const int MPU = 0x68;

float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

// ================= DRIVE PARAMETERS =================
const int BASE_SPEED = 150;
const float KP_GYRO = 10;
const float KP_WALL = 4;

const int MAX_CORRECTION = 80;

// ================= WALL FOLLOW PARAMETERS =================
const int WALL_DISTANCE = 20;   // desired right wall distance
const int FRONT_BLOCK   = 25;
const int RIGHT_OPEN    = 35;

// ================= TURN PARAMETERS =================
const float TARGET_TURN = 87.0;
const int TURN_MIN_SPEED = 200;
const int TURN_MAX_SPEED = 230;

float turnSpeedFactor = 0;

// ================= STATE MACHINE =================
enum RobotState
{
  STATE_NORMAL,
  STATE_TURNING_RIGHT,
  STATE_TURNING_LEFT,
  STATE_TURN_FINISHED,
  STATE_WAIT_AFTER_TURN
};

RobotState currentState = STATE_NORMAL;
unsigned long stateStartTime = 0;


// ================= DISTANCE FUNCTION =================
float getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if(duration == 0)
    return 999;

  return duration * 0.034 / 2;
}


// ================= SETUP =================
void setup()
{
  Serial.begin(9600);

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  Wire.begin();

  // Wake MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Calibrating MPU...");

  long sum = 0;
  for(int i=0;i<2000;i++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU,2,true);
    sum += (int16_t)(Wire.read()<<8 | Wire.read());
    delay(2);
  }

  gyroZ_offset = sum / 2000.0;

  Serial.println("READY");
  previousTime = millis();
}


// ================= LOOP =================
void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime)/1000.0;
  previousTime = currentTime;

  // ===== READ GYRO =====
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU,2,true);
  int16_t GyZ_raw = Wire.read()<<8 | Wire.read();

  float GyZ = (GyZ_raw - gyroZ_offset)/131.0;
  if(abs(GyZ)<0.1) GyZ=0;
  yaw += GyZ*dt;

  // ===== READ DISTANCES =====
  float frontDist = getDistance(trigFront, echoFront);
  float rightDist = getDistance(trigRight, echoRight);
  float leftDist  = getDistance(trigLeft, echoLeft);

  // ================= STATE MACHINE =================
  switch(currentState)
  {

    case STATE_NORMAL:

      // PRIORITY 1: Right side open → Turn Right
      if(rightDist > RIGHT_OPEN)
      {
        yaw = 0;
        turnSpeedFactor = 0;
        currentState = STATE_TURNING_RIGHT;
      }

      // PRIORITY 2: Front blocked → Turn Left
      else if(frontDist < FRONT_BLOCK)
      {
        yaw = 0;
        turnSpeedFactor = 0;
        currentState = STATE_TURNING_LEFT;
      }

    break;


    case STATE_TURNING_RIGHT:
    {
      float remaining = TARGET_TURN - abs(yaw);

      if(remaining <= 0)
      {
        currentState = STATE_TURN_FINISHED;
        stateStartTime = millis();
        break;
      }

      if(abs(yaw) < 30) turnSpeedFactor += 0.02;
      if(remaining < 30) turnSpeedFactor -= 0.04;

      turnSpeedFactor = constrain(turnSpeedFactor,0.4,1.0);
    }
    break;


    case STATE_TURNING_LEFT:
    {
      float remaining = TARGET_TURN - abs(yaw);

      if(remaining <= 0)
      {
        currentState = STATE_TURN_FINISHED;
        stateStartTime = millis();
        break;
      }

      if(abs(yaw) < 30) turnSpeedFactor += 0.02;
      if(remaining < 30) turnSpeedFactor -= 0.04;

      turnSpeedFactor = constrain(turnSpeedFactor,0.4,1.0);
    }
    break;


    case STATE_TURN_FINISHED:
      analogWrite(LeftF,0);
      analogWrite(RightF,0);
      analogWrite(LeftB,0);
      analogWrite(RightB,0);

      currentState = STATE_WAIT_AFTER_TURN;
      stateStartTime = millis();
    break;


    case STATE_WAIT_AFTER_TURN:
      if(millis()-stateStartTime > 300)
      {
        yaw = 0;
        currentState = STATE_NORMAL;
      }
    break;
  }


  // ================= MOTOR CONTROL =================

  if(currentState == STATE_TURNING_RIGHT)
  {
    int turnSpeed = TURN_MIN_SPEED +
      (TURN_MAX_SPEED - TURN_MIN_SPEED)*turnSpeedFactor;

    // Spin right
    analogWrite(LeftF, turnSpeed);
    digitalWrite(LeftB, LOW);

    digitalWrite(RightF, LOW);
    analogWrite(RightB, turnSpeed);
  }

  else if(currentState == STATE_TURNING_LEFT)
  {
    int turnSpeed = TURN_MIN_SPEED +
      (TURN_MAX_SPEED - TURN_MIN_SPEED)*turnSpeedFactor;

    // Spin left
    digitalWrite(LeftF, LOW);
    analogWrite(LeftB, turnSpeed);

    analogWrite(RightF, turnSpeed);
    digitalWrite(RightB, LOW);
  }

  else if(currentState == STATE_NORMAL)
  {
    // ===== WALL FOLLOW CORRECTION =====
    float wallError = WALL_DISTANCE - rightDist;
    float wallCorrection = wallError * KP_WALL;

    float gyroCorrection = yaw * KP_GYRO;

    float correction = wallCorrection + gyroCorrection;
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

    int speedL = BASE_SPEED + correction;
    int speedR = BASE_SPEED - correction;

    speedL = constrain(speedL,0,255);
    speedR = constrain(speedR,0,255);

    analogWrite(LeftF, speedL);
    digitalWrite(LeftB, LOW);

    analogWrite(RightF, speedR);
    digitalWrite(RightB, LOW);
  }

}
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
const int SAFE_DISTANCE = 20;

// ================= TURN =================
const float TARGET_TURN_90  = 75.0;
const float TARGET_TURN_180 = 175.0;
const int TURN_SPEED = 200;

unsigned long stateStartTime = 0;

// ================= FLAGS =================
bool rightTurnSequence = false;
bool leftTurnSequence  = false;
bool deadEndTurn       = false;

// ================= STATES =================
enum RobotState
{
  STATE_NORMAL,
  STATE_STOPPED,
  STATE_TURNING_RIGHT,
  STATE_TURNING_LEFT,
  STATE_TURNING_180,
  STATE_FORWARD_LOCK
};

RobotState currentState = STATE_NORMAL;


// ================= DISTANCE FUNCTION =================
float getDistanceCM(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if(duration == 0) return 999;

  return duration * 0.034 / 2;
}


// ================= SETUP =================
void setup()
{
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

  // Gyro calibration
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

  if(abs(GyZ) < 0.1) GyZ = 0;
  yaw += GyZ * dt;


  switch(currentState)
  {

    // ================= NORMAL (Right Hand Rule) =================
    case STATE_NORMAL:
    {
      float frontDistance = getDistanceCM(trigPin, echoPin);
      float rightDistance = getDistanceCM(trigRight, echoRight);
      float leftDistance  = getDistanceCM(trigLeft, echoLeft);

      bool rightOpen = rightDistance > 25;
      bool frontOpen = frontDistance > SAFE_DISTANCE;
      bool leftOpen  = leftDistance  > 25;

      // 1️⃣ RIGHT PRIORITY
      if(rightOpen)
      {
        digitalWrite(rightLedPin, HIGH);
        rightTurnSequence = true;
        leftTurnSequence  = false;
        deadEndTurn       = false;
        currentState = STATE_STOPPED;
        break;
      }

      // 2️⃣ STRAIGHT
      if(frontOpen)
      {
        float correction = yaw * KP;
        correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

        analogWrite(LeftF, BASE_SPEED + correction);
        digitalWrite(LeftB, LOW);

        analogWrite(RightF, BASE_SPEED - correction);
        digitalWrite(RightB, LOW);

        break;
      }

      // 3️⃣ LEFT
      if(leftOpen)
      {
        digitalWrite(rightLedPin, HIGH);
        rightTurnSequence = false;
        leftTurnSequence  = true;
        deadEndTurn       = false;
        currentState = STATE_STOPPED;
        break;
      }

      // 4️⃣ DEAD END
      rightTurnSequence = false;
      leftTurnSequence  = false;
      deadEndTurn       = true;
      currentState = STATE_STOPPED;
    }
    break;


    // ================= STOP =================
    case STATE_STOPPED:

      analogWrite(LeftF,0);
      analogWrite(RightF,0);
      analogWrite(LeftB,0);
      analogWrite(RightB,0);

      yaw = 0;

      if(deadEndTurn)
          currentState = STATE_TURNING_180;
      else if(rightTurnSequence)
          currentState = STATE_TURNING_RIGHT;
      else if(leftTurnSequence)
          currentState = STATE_TURNING_LEFT;
      else
          currentState = STATE_NORMAL;

    break;


    // ================= RIGHT TURN =================
    case STATE_TURNING_RIGHT:

      if(abs(yaw) >= TARGET_TURN_90)
      {
        analogWrite(LeftF,0);
        analogWrite(RightB,0);

        stateStartTime = millis();
        currentState = STATE_FORWARD_LOCK;
        break;
      }

      digitalWrite(LeftB, LOW);
      digitalWrite(RightF, LOW);
      analogWrite(LeftF, TURN_SPEED);
      analogWrite(RightB, TURN_SPEED);

    break;


    // ================= LEFT TURN =================
    case STATE_TURNING_LEFT:

      if(abs(yaw) >= TARGET_TURN_90)
      {
        analogWrite(LeftB,0);
        analogWrite(RightF,0);

        stateStartTime = millis();
        currentState = STATE_FORWARD_LOCK;
        break;
      }

      digitalWrite(LeftF, LOW);
      digitalWrite(RightB, LOW);
      analogWrite(LeftB, TURN_SPEED);
      analogWrite(RightF, TURN_SPEED);

    break;


    // ================= 180 TURN =================
    case STATE_TURNING_180:

      if(abs(yaw) >= TARGET_TURN_180)
      {
        analogWrite(LeftF,0);
        analogWrite(RightB,0);

        deadEndTurn = false;
        yaw = 0;
        currentState = STATE_NORMAL;
        break;
      }

      digitalWrite(LeftB, LOW);
      digitalWrite(RightF, LOW);
      analogWrite(LeftF, TURN_SPEED);
      analogWrite(RightB, TURN_SPEED);

    break;


    // ================= FORWARD LOCK =================
    case STATE_FORWARD_LOCK:
    {
      float frontDistance = getDistanceCM(trigPin, echoPin);
      int moveSpeed = (frontDistance < 30) ? SLOW_SPEED : BASE_SPEED;

      analogWrite(LeftF, moveSpeed);
      digitalWrite(LeftB, LOW);
      analogWrite(RightF, moveSpeed);
      digitalWrite(RightB, LOW);

      if(millis() - stateStartTime >= 550)
      {
        digitalWrite(rightLedPin, LOW);
        rightTurnSequence = false;
        leftTurnSequence  = false;
        yaw = 0;
        currentState = STATE_NORMAL;
      }
    }
    break;

  }
}
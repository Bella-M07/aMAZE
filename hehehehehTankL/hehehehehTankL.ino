#include <Wire.h>

// ================= MOTOR PINS =================
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

// ================= ULTRASONIC =================
const int trigPin = 7;
const int echoPin = 8;

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

// ================= OBSTACLE =================
const int SAFE_DISTANCE = 33;
const int SLOW_DISTANCE = 44;

// ================= TURN PARAMETERS =================
const float TARGET_TURN = 87.0;

const int TURN_MIN_SPEED = 200;
const int TURN_MAX_SPEED = 230;

float turnSpeedFactor = 0;

unsigned long stateStartTime = 0;


// ================= STATE MACHINE =================
enum RobotState
{
  STATE_NORMAL,
  STATE_SLOWING,
  STATE_STOPPED,

  STATE_WAIT_BEFORE_TURN,
  STATE_TURNING_RIGHT,
  STATE_TURNING_LEFT,     
  STATE_TURN_FINISHED,

  STATE_WAIT_AFTER_TURN,
  STATE_MOVE_AFTER_TURN
};

RobotState currentState = STATE_NORMAL;

float slowdownFactor = 1.0;


// ================= DISTANCE =================
float getDistanceCM()
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

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Wire.begin();

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("CALIBRATING MPU");

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


  float distance = getDistanceCM();


  // ================= STATE MACHINE =================
  switch(currentState)
  {

    case STATE_NORMAL:

      slowdownFactor = 1.0;

      if(distance <= SLOW_DISTANCE)
        currentState = STATE_SLOWING;

    break;



    case STATE_SLOWING:

      slowdownFactor -= 0.04;

      if(distance <= SAFE_DISTANCE)
        slowdownFactor -= 0.08;

      slowdownFactor = constrain(slowdownFactor,0,1);

      if(slowdownFactor<=0.05)
      {
        slowdownFactor=0;
        currentState=STATE_STOPPED;
        stateStartTime=millis();
      }

    break;



    case STATE_STOPPED:

      slowdownFactor=0;
      currentState=STATE_WAIT_BEFORE_TURN;
      stateStartTime=millis();

    break;



    case STATE_WAIT_BEFORE_TURN:

      if(millis()-stateStartTime>600)
      {
        yaw=0;
        turnSpeedFactor=0;
        currentState=STATE_TURNING_LEFT;
      }

    break;

    case STATE_TURNING_LEFT:
    {
      float remaining = TARGET_TURN - abs(yaw);

      if(remaining<=0)
      {
        currentState=STATE_TURN_FINISHED;
        stateStartTime=millis();
        break;
      }

      if(abs(yaw)<30)
        turnSpeedFactor+=0.02;

      if(remaining<30)
        turnSpeedFactor-=0.04;

      turnSpeedFactor=constrain(turnSpeedFactor,0.4,1.0);
    }
    break;

    case STATE_TURNING_RIGHT:
    {
      float remaining = TARGET_TURN - abs(yaw);

      if(remaining<=0)
      {
        currentState=STATE_TURN_FINISHED;
        stateStartTime=millis();
        break;
      }

      if(abs(yaw)<30)
        turnSpeedFactor+=0.02;

      if(remaining<30)
        turnSpeedFactor-=0.04;

      turnSpeedFactor=constrain(turnSpeedFactor,0.4,1.0);
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

      if(millis()-stateStartTime>500)
      {
        yaw = 0;
        currentState = STATE_MOVE_AFTER_TURN;
      }

    break;



    case STATE_MOVE_AFTER_TURN:

      slowdownFactor = 1.0;
      currentState = STATE_NORMAL;

    break;

  }



  // ================= MOTOR CONTROL =================

  if(currentState==STATE_TURNING_LEFT)
  {
    int turnSpeed =
    TURN_MIN_SPEED +
    (TURN_MAX_SPEED - TURN_MIN_SPEED)*turnSpeedFactor;

    // --- TANK LEFT TURN (SWAPPED) ---
    // Left side pulls BACKWARD
    digitalWrite(LeftF, LOW);
    analogWrite(LeftB, turnSpeed);

    // Right side pushes FORWARD
    analogWrite(RightF, turnSpeed);
    digitalWrite(RightB, LOW);
  }

  else if(currentState==STATE_TURNING_RIGHT)
  {
    int turnSpeed =
    TURN_MIN_SPEED +
    (TURN_MAX_SPEED - TURN_MIN_SPEED)*turnSpeedFactor;

    // --- TANK RIGHT TURN ---
    // Left Forward
    analogWrite(LeftF, turnSpeed);
    digitalWrite(LeftB, LOW);

    // Right Backward
    digitalWrite(RightF, LOW);
    analogWrite(RightB, turnSpeed);
  }


  else if(currentState==STATE_NORMAL ||
          currentState==STATE_SLOWING ||
          currentState==STATE_MOVE_AFTER_TURN)
  {

    float correction=0;

    if(abs(yaw)>0.3)
    {
      correction=yaw*KP;

      if(correction>0) correction+=MIN_KICK;
      if(correction<0) correction-=MIN_KICK;

      correction=constrain(correction,
                           -MAX_CORRECTION,
                           MAX_CORRECTION);
    }

    int speedL=BASE_SPEED+correction;
    int speedR=BASE_SPEED-correction;

    int finalSpeedL=speedL*slowdownFactor;
    int finalSpeedR=speedR*slowdownFactor;

    analogWrite(LeftF,finalSpeedL);
    digitalWrite(LeftB,LOW);

    analogWrite(RightF,finalSpeedR);
    digitalWrite(RightB,LOW);

  }

}
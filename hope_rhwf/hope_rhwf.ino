#include <Wire.h>

// ================= IBT_2 MOTOR PINS =================
const int Right_RPWM = 3; 
const int Right_LPWM = 9; 
const int Left_RPWM  = 10; 
const int Left_LPWM  = 11; 

// ================= ULTRASONIC PINS =================
const int trigFront = 7;  const int echoFront = 8;
const int trigRight = 5;  const int echoRight = 6;
const int trigLeft  = 4;  const int echoLeft  = 2;

// ================= MPU-6050 =================
const int MPU = 0x68;
float gyroZ_offset = 0;
float yaw = 0;
unsigned long previousTime;

// ================= DRIVE PARAMETERS =================
const int BASE_SPEED = 140;     
const float KP_GYRO = 10;       // Straight line stability
const int TURN_PWM = 250;       // Fast turn speed as requested

// ================= WALL FOLLOWING PARAMETERS =================
const int DESIRED_DIST = 18;    // Stay 18cm from right wall
const int WALL_THRESHOLD = 35;  // If distance > 35, wall is missing (corner)
const int FRONT_STOP_DIST = 25; // Distance to stop and turn left

// ================= STATE MACHINE =================
enum RobotState {
  STATE_FOLLOWING,
  STATE_TURN_LEFT_90,
  STATE_TURN_RIGHT_90,
  STATE_STOPPED
};

RobotState currentState = STATE_FOLLOWING;
float TARGET_YAW = 0;
unsigned long stateStartTime = 0;

// ================= DISTANCE FUNCTION =================
float getDist(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  pinMode(Right_RPWM, OUTPUT); pinMode(Right_LPWM, OUTPUT);
  pinMode(Left_RPWM, OUTPUT);  pinMode(Left_LPWM, OUTPUT);
  
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  long sum = 0;
  for (int i = 0; i < 1500; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2);
  }
  gyroZ_offset = sum / 1500.0;
  previousTime = millis();
}

// ================= LOOP =================
void loop() {
  // 1. UPDATE GYRO
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  int16_t GyZ_raw = Wire.read() << 8 | Wire.read();
  float GyZ = (GyZ_raw - gyroZ_offset) / 131.0;
  if (abs(GyZ) < 0.15) GyZ = 0;
  yaw += GyZ * dt;

  // 2. READ SENSORS
  float distF = getDist(trigFront, echoFront);
  float distR = getDist(trigRight, echoRight);

  // 3. STATE MACHINE
  switch (currentState) {
    
    case STATE_FOLLOWING:
      // Check for wall in front
      if (distF < FRONT_STOP_DIST) {
        stopMotors();
        yaw = 0;
        TARGET_YAW = 85.0; // Prep for Left 90
        currentState = STATE_TURN_LEFT_90;
        stateStartTime = millis();
      } 
      // Check if Right wall disappeared (Open corner)
      else if (distR > WALL_THRESHOLD) {
        stopMotors();
        delay(200); // Move slightly forward before turning
        yaw = 0;
        TARGET_YAW = 85.0; // Prep for Right 90
        currentState = STATE_TURN_RIGHT_90;
      }
      else {
        // Wall following logic
        float error = distR - DESIRED_DIST;
        float correction = error * 8; // PD style correction
        
        drive(BASE_SPEED + correction, BASE_SPEED - correction);
      }
      break;

    case STATE_TURN_LEFT_90:
      // LEFT TURN: Right Forward, Left Backward
      drive(TURN_PWM, -TURN_PWM);
      if (abs(yaw) >= TARGET_YAW) {
        stopMotors();
        currentState = STATE_FOLLOWING;
        yaw = 0;
      }
      break;

    case STATE_TURN_RIGHT_90:
      // RIGHT TURN: Left Forward, Right Backward
      drive(-TURN_PWM, TURN_PWM);
      if (abs(yaw) >= TARGET_YAW) {
        stopMotors();
        currentState = STATE_FOLLOWING;
        yaw = 0;
      }
      break;
  }
}

// ================= IBT_2 HELPER FUNCTIONS =================

void drive(int speedL, int speedR) {
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);

  // Left Motor
  if (speedL >= 0) {
    analogWrite(Left_RPWM, speedL);
    analogWrite(Left_LPWM, 0);
  } else {
    analogWrite(Left_RPWM, 0);
    analogWrite(Left_LPWM, abs(speedL));
  }

  // Right Motor
  if (speedR >= 0) {
    analogWrite(Right_RPWM, speedR);
    analogWrite(Right_LPWM, 0);
  } else {
    analogWrite(Right_RPWM, 0);
    analogWrite(Right_LPWM, abs(speedR));
  }
}

void stopMotors() {
  analogWrite(Left_RPWM, 0);  analogWrite(Left_LPWM, 0);
  analogWrite(Right_RPWM, 0); analogWrite(Right_LPWM, 0);
}
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- Pin Definitions ---
const int RightF = 3;  const int RightB = 9;
const int LeftF = 10; const int LeftB = 11;

// --- Power Settings ---
const int TURN_SPEED = 155;    
const int KICK_PULSE = 255;    

// --- Navigation Variables ---
float yawAngle = 0;
float gzBias = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);

  // --- 1. Calibrate Gyro ---
  Serial.println("Calibrating... DO NOT MOVE");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;

  // --- 2. The Specific Right Turn Kickstart ---
  delay(2000); 
  Serial.println("Kickstarting Right Turn...");

  // To turn RIGHT: Left goes FORWARD, Right goes BACKWARD
  analogWrite(LeftF, KICK_PULSE);
  analogWrite(RightB, KICK_PULSE);
  analogWrite(LeftB, 0);
  analogWrite(RightF, 0);
  
  delay(80); // Powerful burst to get those gears moving

  // Drop to turning speed and let the IMU take over
  turnToAngle(-45); 

  stopMotors();
  Serial.println("Turn Complete.");
}

void loop() {}

void turnToAngle(float target) {
  lastTime = millis(); 

  while (true) {
    updateYaw();
    float error = target - yawAngle;

    // Exit condition
    if (abs(error) < 2.0) {
      stopMotors();
      break; 
    }

    // Maintain movement at TURN_SPEED
    if (error < 0) { // Turning Right
      analogWrite(LeftF, TURN_SPEED);
      analogWrite(RightB, TURN_SPEED);
      analogWrite(LeftB, 0);
      analogWrite(RightF, 0);
    } else { // Turning Left (if needed)
      analogWrite(RightF, TURN_SPEED);
      analogWrite(LeftB, TURN_SPEED);
      analogWrite(RightB, 0);
      analogWrite(LeftF, 0);
    }
    
    Serial.print("Yaw: "); Serial.println(yawAngle);
    delay(10);
  }
}

void updateYaw() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  yawAngle += ((gz - gzBias) / 131.0) * dt;
}

void stopMotors() {
  analogWrite(LeftF, 0);  analogWrite(LeftB, 0);
  analogWrite(RightF, 0); analogWrite(RightB, 0);
}
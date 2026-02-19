#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- Pin Definitions ---
const int RightF = 3;  const int RightB = 9;
const int LeftF = 10; const int LeftB = 11;
const int trigPin = A1;
const int echoPin = A0;

// --- Variables ---
float yawAngle = 0;
float gzBias = 0;
unsigned long lastTime = 0;
long duration;
int distance;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // --- Calibrate Gyro (Keep robot still!) ---
  Serial.println("Calibrating Gyro... Do not move.");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;
  Serial.print("Bias Calculated: "); Serial.println(gzBias);

  // --- Kickstart ---
  analogWrite(RightF, 255);
  analogWrite(LeftF, 255);
  delay(100);

  lastTime = millis();
}

void loop() {
  // 1. Get IMU Data & Calculate Yaw
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  
  // Convert raw gyro to degrees/sec and integrate
  yawAngle += ((gz - gzBias) / 131.0) * dt;

  // 2. Get Ultrasonic Distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  distance = duration * 0.034 / 2;

  // 3. Define Default Speeds
  int speedR = 75;
  int speedL = 75;

  // 4. Combined Logic
  // Check for obstacle first (Priority 1)
  if (distance > 0 && distance <= 25) {
    speedR = 0;
    speedL = 0;
    Serial.print("OBSTACLE! ");
  } 

  else {
    if (yawAngle > 4.0) {       // Drifting Right -> Turn Left
      speedL = 90;   // <--- SWAP THESE
      speedR = 65; 
    } else if (yawAngle < -4.0) { // Drifting Left -> Turn Right
      speedL = 65;   // <--- SWAP THESE
      speedR = 90; 
    }
  }
  // // Straight-line correction if path is clear (Priority 2)
  // else {
  //   if (yawAngle > 2.0) {       // Drifting Right -> Turn Left
  //     speedL = 65; 
  //     speedR = 90; 
  //   } else if (yawAngle < -2.0) { // Drifting Left -> Turn Right
  //     speedL = 90; 
  //     speedR = 65; 
  //   }
  // }

  // 5. Apply Power
  analogWrite(LeftF, speedL);
  analogWrite(RightF, speedR);

  // 6. Debugging
  Serial.print("Dist: "); Serial.print(distance);
  Serial.print("cm | Yaw: "); Serial.print(yawAngle);
  Serial.print(" | L/R: "); Serial.print(speedL);
  Serial.print("/"); Serial.println(speedR);

  delay(10); // Faster loop for better gyro integration
}
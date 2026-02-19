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
float targetAngle = 0; // The angle the robot is currently trying to maintain
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

  // --- Calibrate Gyro (Robot must be perfectly still!) ---
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

  // Kickstart
  analogWrite(RightF, 255);
  analogWrite(LeftF, 255);
  delay(100);

  lastTime = millis();
}

void loop() {
  // 1. Update Yaw Angle using Gyro
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  yawAngle += ((gz - gzBias) / 131.0) * dt;

  // 2. Check Distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000); 
  distance = duration * 0.034 / 2;

  // 3. Define Base Speeds
  int speedR = 75;
  int speedL = 75;

  // 4. OBSTACLE HANDLING (The Turning Trigger)
  if (distance > 0 && distance <= 20) {
    Serial.println("OBSTACLE DETECTED! Performing Turn...");
    
    // Stop briefly
    analogWrite(LeftF, 0); analogWrite(RightF, 0);
    delay(500);
    
    // Update target angle (Subtract 90 for a Right turn, Add 90 for a Left turn)
    targetAngle -= 90.0; 
    
    // We stay in the loop, the PID logic below will now force the turn 
    // to reach the new targetAngle.
  }

  // 5. GYRO STEERING LOGIC (The "Straight" and "Turning" engine)
  // Calculate the error between where we are and where we want to be
  float error = targetAngle - yawAngle;
  
  // Proportional correction: 
  // If error is positive, we need to turn one way. If negative, the other.
  // We use a multiplier (4.0) to determine how aggressively it turns.
  float correction = error * 4.0; 
  
  // Constrain correction so motors don't go over 255 or under 0
  correction = constrain(correction, -50, 50);

  // Apply correction to motors
  // If the robot spins the wrong way, swap the '+' and '-' signs here
  speedL = 75 + correction;
  speedR = 75 - correction;

  // 6. Apply Power
  analogWrite(LeftF, speedL);
  analogWrite(RightF, speedR);

  // 7. Debugging
  Serial.print("Target: "); Serial.print(targetAngle);
  Serial.print(" | Yaw: "); Serial.print(yawAngle);
  Serial.print(" | Error: "); Serial.println(error);

  delay(10); 
}
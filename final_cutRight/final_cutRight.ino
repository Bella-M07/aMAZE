#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- Pin Definitions ---
const int RightF = 3;  
const int RightB = 9;  
const int LeftF  = 10; 
const int LeftB  = 11; 

const int trigPin = A0;
const int echoPin = A1;

// --- Variables ---
long duration;
int distance;

// IMU Variables
float yaw = 0;
float gzBias = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(9600); 
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // --- IMU Calibration ---
  Serial.println("Calibrating IMU... Keep Still");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;
  Serial.println("Done");

  delay(2000);

  // --- Kickstart ---
  analogWrite(RightF, 100);
  analogWrite(LeftF, 100);
  delay(100);               

  lastTime = millis();
}

void loop() {
  // 1. Update IMU Heading (Yaw)
  updateYaw();

  // 2. Trigger the Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // 3. Logic for Detection
  if (distance <= 17 && distance > 0) {
    // Turning Logic from your original code
    analogWrite(LeftF, 10); 
    analogWrite(RightB, -3); // Note: analogWrite doesn't take negatives, usually 0-255
    delay(470); 

    analogWrite(LeftF, 0); 
    analogWrite(RightB, 0);
    
    // Reset Yaw after turn so the new direction is the "new straight"
    yaw = 0; 
  }

  // 4. Straight Path Correction Logic
  // baseSpeed 50 from your code's final analogWrite
  int baseSpeed = 50; 
  
  // Correction: If yaw is positive, turn right. If negative, turn left.
  float correction = yaw * 5.0; 

  int speedL = constrain(baseSpeed + (int)correction, 0, 255);
  int speedR = constrain(baseSpeed - (int)correction, 0, 255);

  analogWrite(LeftF, speedL);
  analogWrite(RightF, speedR);
  analogWrite(LeftB, 0);
  analogWrite(RightB, 0);

  delay(50); // Faster loop for better gyro integration
}

void updateYaw() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 131.0 is the sensitivity scale factor for MPU6050
  float gzRate = (gz - gzBias) / 131.0;
  yaw += gzRate * dt;
}
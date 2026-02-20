#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- Pins ---
const int RightF = 3;  const int RightB = 9;
const int LeftF = 10; const int LeftB = 11;
const int trigPin = A1;
const int echoPin = A0;

// --- Variables ---
float yaw = 0;
float gzBias = 0;
unsigned long lastTime = 0;
int distance;

// UPDATED: Positive 90 for Left Turn
float targetAngle = 90;    
float tolerance = 2;       

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  // Calibration
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;
  
  // Initial Kickstart
  analogWrite(RightF, 255); analogWrite(LeftF, 255);
  delay(100);
  lastTime = millis();
}

void loop() {
  updateYaw(); 
  
  // Distance Check
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); 
  distance = duration * 0.034 / 2;

  if (distance > 0 && distance <= 20) { 
    executePrecisionLeftTurn();
  } 
  else {
    // --- STRAIGHT LINE LOGIC ---
    int baseSpeed = 80; 
    float correction = yaw * 5.0; 

    analogWrite(LeftF, constrain(baseSpeed + (int)correction, 0, 255));
    analogWrite(RightF, constrain(baseSpeed - (int)correction, 0, 255));
    analogWrite(LeftB, 0);
    analogWrite(RightB, 0);
  }
}

void updateYaw() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  float gzRate = (gz - gzBias) / 131.0;
  yaw += gzRate * dt;
}

void executePrecisionLeftTurn() {
  // 1. Full Stop
  analogWrite(LeftF, 0); analogWrite(RightF, 0);
  analogWrite(LeftB, 0); analogWrite(RightB, 0);
  delay(500);
  
  // 2. Pivot Turn with Correction Logic
  yaw = 0; 
  lastTime = millis();
  bool turning = true;

  while (turning) {
    updateYaw();
    float error = targetAngle - yaw;

    if (abs(error) <= tolerance) {
      turning = false; 
    }
    // UPDATED: If error is positive, we still need to reach +90
    else if (error > 0) {
      turnLeft();
    }
    // If error is negative, we overshot +90
    else {
      turnRight();
    }
    delay(5);
  }
  
  // 3. STRONG ACTIVE BRAKE (Right Turn burst to stop Left spin)
  analogWrite(LeftF, 255);  analogWrite(RightB, 255);
  analogWrite(LeftB, 0);    analogWrite(RightF, 0); 
  delay(80); 
  
  // 4. STOP & SETTLE
  analogWrite(LeftF, 0); analogWrite(RightB, 0);
  delay(1000);   
  yaw = 0;  
  
  // 5. Kickstart forward
  analogWrite(LeftF, 255); analogWrite(RightF, 255);
  delay(120);
  lastTime = millis(); 
}

void turnRight() {
  analogWrite(LeftF, 220);
  analogWrite(RightB, 220);
  analogWrite(LeftB, 0);
  analogWrite(RightF, 0);
}

void turnLeft() {
  analogWrite(RightF, 220);
  analogWrite(LeftB, 220);
  analogWrite(LeftF, 0);
  analogWrite(RightB, 0);
}
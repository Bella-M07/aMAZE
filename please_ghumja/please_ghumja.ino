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

// Exact values for the correction logic
float targetAngle = -90;   // Right turn target
float tolerance = 2;       // Allowed error (+/- 2 degrees)

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

  analogWrite(LeftF, 255); analogWrite(RightF, 255); 
  delay(500);

   analogWrite(LeftF, 75); analogWrite(RightF, 75); 
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
    executePrecisionTurn();
  } 
  else {
    // --- STRAIGHT LINE LOGIC ---
    int baseSpeed = 75; 
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

// --- STRUCTURE OF CODE 2 WITH LOGIC OF CODE 1 ---
void executePrecisionTurn() {
  // 1. Full Stop
  analogWrite(LeftF, 0); analogWrite(RightF, 0);
  analogWrite(LeftB, 0); analogWrite(RightB, 0);
  delay(500);
  
  // 2. Pivot Turn with Correction Logic
  yaw = 0; // Start turn from 0
  lastTime = millis();
  bool turning = true;

  while (turning) {
    updateYaw();
    float error = targetAngle - yaw;

    // Check if we are within the 2-degree tolerance
    if (abs(error) <= tolerance) {
      turning = false; // This exits the while loop
    }
    // If we haven't reached -90 yet -> Turn Right
    else if (error < 0) {
      turnRight();
    }
    // If we passed -90 -> Turn Left to fix it
    else {
      turnLeft();
    }
    delay(5);
  }
  
  // 3. STRONG ACTIVE BRAKE
  // (Firing motors in opposite direction to stop instantly)
  // analogWrite(LeftF, 0);   analogWrite(RightB, 0);
  // analogWrite(LeftB, 255); analogWrite(RightF, 255); 
  // delay(80); 
  
  // 4. STOP & SETTLE
  analogWrite(LeftB, 0); analogWrite(RightF, 0);
  delay(1000);   
  yaw = 0;  // Reset for the new forward direction
  
  // 5. Kickstart forward
  analogWrite(LeftF, 255); analogWrite(RightF, 255);
  delay(120);
  lastTime = millis(); 
}

// Helper functions for the correction logic
void turnRight() {
  analogWrite(LeftF, 100);
  analogWrite(RightB, -75);
  analogWrite(LeftB, 0);
  analogWrite(RightF, 0);
}

void turnLeft() {
  analogWrite(RightF,100);
  analogWrite(LeftB, -75);
  analogWrite(LeftF, 0);
  analogWrite(RightB, 0);
}
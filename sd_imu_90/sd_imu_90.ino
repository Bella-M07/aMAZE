#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- Pins ---
const int RightF = 3;  const int RightB = 9;
const int LeftF = 10; const int LeftB = 11;
const int trigPin = A1;
const int echoPin = A0;

// --- Variables ---
float yawAngle = 0;
float gzBias = 0;
unsigned long lastTime = 0;
int distance;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  // Calibration (Keep robot still!)
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
    execute90DegreeTurn();
  } 
  else {
    // --- STRAIGHT LINE LOGIC ---
    int baseSpeed = 80; 
    
    // CRITICAL: If the robot spirals, change 10.0 to -10.0
    float correction = yawAngle * 5.0; 

    // We apply correction to BOTH motors to pivot back to center
    int speedL = baseSpeed + (int)correction;
    int speedR = baseSpeed - (int)correction;

    analogWrite(LeftF, constrain(speedL, 0, 255));
    analogWrite(RightF, constrain(speedR, 0, 255));
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
  
  // Gyro sensitivity for MPU6050 is 131.0 at default range
  yawAngle += ((gz - gzBias) / 131.0) * dt;
}

void execute90DegreeTurn() {
  // 1. Full Stop
  analogWrite(LeftF, 0); analogWrite(RightF, 0);
  analogWrite(LeftB, 0); analogWrite(RightB, 0);
  delay(500);
  
  // 2. Pivot Turn
  // If it's turning 160, we need to stop the motors MUCH earlier.
  // We will aim for only 45-50 degrees and let momentum do the rest.
  float targetTurn = yawAngle - 35.0; 

  while (yawAngle > targetTurn) {
    updateYaw();
    analogWrite(LeftF, 180); // Lowered power slightly for more control
    analogWrite(RightB, 180);
    delay(5);
  }
  
  // 3. STRONG ACTIVE BRAKE
  // We fire the opposite motors at full power for a moment to "clamped" the robot
  analogWrite(LeftF, 0);   analogWrite(RightB, 0);
  analogWrite(LeftB, 255); analogWrite(RightF, 255); 
  delay(80); // Increased brake duration from 50ms to 80ms
  
  // 4. STOP & SETTLE
  analogWrite(LeftB, 0); analogWrite(RightF, 0);
  delay(1000);   // Longer pause to ensure all movement has stopped
  yawAngle = 0;  // Now we reset to zero
  
  // 5. Kickstart forward
  analogWrite(LeftF, 255); analogWrite(RightF, 255);
  delay(120);
  lastTime = millis(); 
}
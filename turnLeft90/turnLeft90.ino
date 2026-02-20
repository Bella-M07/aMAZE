#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- Pins ---
const int RightF = 3;  const int RightB = 9;
const int LeftF = 10; const int LeftB = 11;

// --- Variables ---
float yawAngle = 0;
float gzBias = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);

  // 1. Calibration (Keep still!)
  Serial.println("Calibrating... DO NOT MOVE");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;
  Serial.println("Calibration Done.");
  
  delay(1000); // 2 second pause before movement

  // 2. Execute the Left Turn
  Serial.println("Turning Left...");
  performLeft90Turn();
  
  Serial.println("Turn Complete.");
}

void loop() {
  // Empty
}

void performLeft90Turn() {
  lastTime = millis();
  
  // 1. Kickstart (To fix the jerkiness/humming)
  analogWrite(RightF, 255); 
  analogWrite(LeftB, 255);
  delay(50); // Short burst to break friction

  // 2. Pivot Turn (Target +35 for 90 degrees)
  float targetTurn = 50.5; 

  while (yawAngle < targetTurn) {
    updateYaw();
    // Pivot Left: Right Forward, Left Backward
    analogWrite(RightF, 190); 
    analogWrite(LeftB, 190);
    analogWrite(LeftF, 0);
    analogWrite(RightB, 0);
    delay(5);
  }

  // 3. STRONG ACTIVE BRAKE (To kill momentum)
  analogWrite(RightF, 0);   analogWrite(LeftB, 0);
  analogWrite(LeftF, 255);  analogWrite(RightB, 255); 
  delay(80); 
  
  // 4. FULL STOP
  analogWrite(LeftF, 0); analogWrite(RightB, 0);
}

void updateYaw() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  yawAngle += ((gz - gzBias) / 131.0) * dt;
}
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

// Exact values from your logic
float targetAngle = -90;   // Right turn target
float tolerance = 2;       // Allowed error (+/- 2 degrees)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  // Calibration (from your logic)
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;
  
  lastTime = millis();

  analogWrite(LeftF, 255);
  analogWrite(RightF, 255);  
  delay(100);
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
    executeObstacleAvoidanceTurn();
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

// --- YOUR EXACT TURN LOGIC INTEGRATED HERE ---
void executeObstacleAvoidanceTurn() {
  stopMotors();
  delay(500);
  
  yaw = 0; // Reset yaw before the turn
  lastTime = millis();
  bool turning = true;

  while (turning) {
    updateYaw();
    float error = targetAngle - yaw;

    Serial.print("Yaw: ");
    Serial.println(yaw);

    if (abs(error) <= tolerance) {
      stopMotors();
      Serial.println("Turn Complete");
      turning = false; // Exit the turning loop
    }
    // If we haven't reached -90 yet → turn right
    else if (error < 0) {
      turnRight(120);
    }
    // If we passed -90 → turn left to correct
    else {
      turnLeft(100);
    }
    delay(5);
  }

  // Brief delay after turn before resuming forward path
  delay(1000);
  yaw = 0;             // Reset yaw to 0 for the new straight path
  lastTime = millis(); // Reset timing
}

void turnRight(int speed) {
  analogWrite(LeftF, speed);
  analogWrite(RightB, speed);
  analogWrite(LeftB, 0);
  analogWrite(RightF, 0);
}

void turnLeft(int speed) {
  analogWrite(RightF, speed);
  analogWrite(LeftB, speed);
  analogWrite(LeftF, 0);
  analogWrite(RightB, 0);
}

void stopMotors() {
  analogWrite(LeftF, 0);
  analogWrite(LeftB, 0);
  analogWrite(RightF, 0);
  analogWrite(RightB, 0);
}
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor Pins
const int RightF = 3;
const int RightB = 9;
const int LeftF  = 10;
const int LeftB  = 11;

float yaw = 0;
float gzBias = 0;
unsigned long lastTime;

float targetAngle = -90;   // Right turn target
float tolerance = 2;       // Allowed error (+/- 2 degrees)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  Serial.println("Calibrating... Keep still");

  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;

  Serial.println("Done");
  delay(1000);

  yaw = 0;
  lastTime = millis();
}

void loop() {

  updateYaw();

  float error = targetAngle - yaw;

  Serial.print("Yaw: ");
  Serial.println(yaw);

  if (abs(error) <= tolerance) {
    stopMotors();
    Serial.println("Turn Complete");
    while (1);   // stop program
  }

  // If we haven't reached -90 yet → turn right
  if (error < 0) {
    turnRight(120);
  }
  // If we passed -90 → turn left to correct
  else {
    turnLeft(100);
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
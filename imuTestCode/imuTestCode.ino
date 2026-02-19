#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int RightF = 3;
const int LeftF = 10;

float yawAngle = 0;
float gzBias = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(RightF, OUTPUT);
  pinMode(LeftF, OUTPUT);

  // Calibrate — keep still during this!
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gzBias = sum / 500.0;
  Serial.print("Bias: "); Serial.println(gzBias);

  // Kickstart
  analogWrite(RightF, 255);
  analogWrite(LeftF, 255);
  delay(100);

  lastTime = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  yawAngle += ((gz - gzBias) / 131.0) * dt;

  int speedR = 75;
  int speedL = 75;

  if (yawAngle > 2.0) {
    speedL = 85;
    speedR = 65;
  } else if (yawAngle < -2.0) {
    speedR = 85;
    speedL = 65;
  }

  analogWrite(LeftF, speedL);
  analogWrite(RightF, speedR);

  Serial.print("Yaw: "); Serial.print(yawAngle);
  Serial.print(" | L: "); Serial.print(speedL);
  Serial.print(" R: "); Serial.println(speedR);

  delay(20);
}
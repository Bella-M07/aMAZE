// --- Pin Definitions ---
// Motors
const int RightF = 3;  // Forward PWM
const int RightB = 9;  // Reverse PWM
const int LeftF = 10;  // Forward PWM
const int LeftB = 11;  // Reverse PWM

// Ultrasonic Sensor
const int trigL = A5; const int echoL = A4; //left
const int trigC = A1; const int echoC = A0; //center
const int trigR = A3; const int echoR = A2; //Right

// --- Variables ---
long duration;
int distC;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600); 
  
  // Set Pin Modes
  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);

  //ultrasonic
  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT); pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT); pinMode(echoR, INPUT);

  // --- Robot Movement Initialization ---
  // Kickstart: Full speed (255) for 100ms to overcome static friction
  analogWrite(RightF, 255);
  analogWrite(LeftF, 255);
  delay(100);               

  // Cruising: Drop to 75 speed for the remainder of the run
  analogWrite(RightF, 75);
  analogWrite(LeftF, 75);
}

void loop() {

  long distL = getDistance(trigL, echoL);
  delay(20); // Short delay to prevent sound interference
  long distC = getDistance(trigC, echoC);
  delay(20);
  long distR = getDistance(trigR, echoR);
  delay(20);

  // 5. Logic for Detection
  if (distC <= 300 && distC > 0) {
    Serial.println("--- Object Detected within 5cm! ---");
    // Optional: Add code here to stop the robot if it gets too close
    analogWrite(RightF, 200);
    analogWrite(LeftF, 200);
    if (distC <= 200)
    {
      analogWrite(RightF, 0);
      analogWrite(LeftF, 0);
    }
  }

  //6.Logic for LR-PID
  if (distL <= 50 && distL > 0) {
    while(distL <=60 && distR <=50)
    {
      analogWrite(LeftF, 80);
      distL = getDistance(trigL, echoL);
    }
  }

  if (distR <= 50 && distR > 0) {
    while(distR <=60 && distL <= 50)
    {
      analogWrite(RightF, 80);
      distR = getDistance(trigR, echoR);
    }
  }

  // Small delay to stabilize readings
  delay(200);
}

long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  long duration = pulseIn(echo, HIGH);
  long distance = duration * 0.343 / 2.0; // Speed of sound math
  return distance;
}
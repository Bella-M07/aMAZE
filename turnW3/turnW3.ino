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
  delay(10); // Short delay to prevent sound interference
  long distC = getDistance(trigC, echoC);
  delay(10);
  long distR = getDistance(trigR, echoR);
  delay(10);

  int speedR=75;
  int speedL=75;

  // 5. Logic for Detection
  if (distC <= 250 && distC > 0) {
    Serial.println("--- Object Detected within 5cm! ---");
    // Optional: Add code here to stop the robot if it gets too close
    // analogWrite(RightF, 20);
    // analogWrite(LeftF, 20);
    // if (distC <= 200)
    // {
    //   analogWrite(RightF, 0);
    //   analogWrite(LeftF, 0);
    // }
    speedR=0;
    speedL=0;
  }

  else if (distL <= 55 && distL > 44) {
    // analogWrite(LeftF, 80);
    speedL=80;
  }

  else if (distR <= 55 && distR > 44) {
    // analogWrite(RightF, 80);
    speedR=80;
  }

  else if (distR > 150)
  {
    // analogWrite(LeftF, 80);
    speedL=80;
  }

  else if (distL > 150)
  {
    // analogWrite()
    speedR=80;
  }

  analogWrite(LeftF,speedL);
  analogWrite(RightF,speedR);
  // Small delay to stabilize readings

  delay(20);
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
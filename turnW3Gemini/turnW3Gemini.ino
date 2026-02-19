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
  long distL = getDistance(trigL, echoL); delay(10);
  long distC = getDistance(trigC, echoC); delay(10);
  long distR = getDistance(trigR, echoR); delay(10);

  int speedR = 75; // Default Cruise
  int speedL = 75; // Default Cruise

  // Priority 1: STOP if blocked in front
  if (distC > 0 && distC <= 250) {
    speedL = 0;
    speedR = 0;
    Serial.println("BLOCKED");
  } 
  // Priority 2: Too close to a wall (Hard Turn)
  else if (distL > 0 && distL < 60) {
    speedL = 110; // Push away from left
    speedR = 40;
    Serial.println("Too Left - Turning Right");
  }
  else if (distR > 0 && distR < 60) {
    speedL = 40;
    speedR = 110; // Push away from right
    Serial.println("Too Right - Turning Left");
  }
  // Priority 3: Drifting away (Soft Correction)
  else if (distR > 180) {
    speedL = 95; 
    speedR = 75;
  }
  else if (distL > 180) {
    speedR = 95;
    speedL = 75;
  }

  analogWrite(LeftF, speedL);
  analogWrite(RightF, speedR);
  
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


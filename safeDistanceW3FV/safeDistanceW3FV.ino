// --- Pin Definitions ---
// Motors
const int RightF = 3;  // Forward PWM
const int RightB = 9;  // Reverse PWM
const int LeftF = 10;  // Forward PWM
const int LeftB = 11;  // Reverse PWM

// Ultrasonic Sensors
const int trigL = A5; const int echoL = A4; // Left
const int trigC = A1; const int echoC = A0; // Center
const int trigR = A3; const int echoR = A2; // Right

// --- Constants & Settings ---
const int BASE_SPEED = 75;      // Normal cruising speed
const int KICKSTART = 255;      // Burst to overcome friction
const int TURN_ADJUST = 40;     // Strength of steering correction
const int MIN_DIST_MM = 200;    // Stop if Center is closer than 15cm (150mm)
const int WALL_BUFFER_MM = 50; // Stay at least 10cm from side walls

void setup() {
  Serial.begin(9600); 
  
  // Set Motor Pin Modes
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);

  // Set Sensor Pin Modes
  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT); pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT); pinMode(echoR, INPUT);

  // --- Robot Kickstart ---
  // Give a 100ms burst to get motors turning
  analogWrite(RightF, KICKSTART);
  analogWrite(LeftF, KICKSTART);
  delay(100); 

  // Initial drop to cruising speed
  analogWrite(RightF, BASE_SPEED);
  analogWrite(LeftF, BASE_SPEED);
}

void loop() {
  // 1. Get Distances in mm
  long distL = getDistance(trigL, echoL);
  delay(15); // Small delay to prevent ultrasonic cross-talk
  long distC = getDistance(trigC, echoC);
  delay(15);
  long distR = getDistance(trigR, echoR);

  // 2. Debugging Output
  Serial.print("L: "); Serial.print(distL);
  Serial.print(" | C: "); Serial.print(distC);
  Serial.print(" | R: "); Serial.println(distR);

  // 3. COLLISION PREVENTION (Center Sensor)
  if (distC > 0 && distC <= MIN_DIST_MM) {
    // Immediate Stop
    analogWrite(RightF, 0);
    analogWrite(LeftF, 0);
    Serial.println("!!! EMERGENCY STOP !!!");
  } 
  
  // 4. HALLWAY CENTERING (Left/Right Logic)
  else {
    // If too close to LEFT wall -> Turn RIGHT
    if (distL > 0 && distL < WALL_BUFFER_MM) {
      analogWrite(LeftF, BASE_SPEED + TURN_ADJUST); 
      analogWrite(RightF, BASE_SPEED - TURN_ADJUST);
      Serial.println("Steering Right...");
    } 
    // If too close to RIGHT wall -> Turn LEFT
    else if (distR > 0 && distR < WALL_BUFFER_MM) {
      analogWrite(LeftF, BASE_SPEED - TURN_ADJUST);
      analogWrite(RightF, BASE_SPEED + TURN_ADJUST);
      Serial.println("Steering Left...");
    } 
    // Otherwise, drive straight
    else {
      analogWrite(LeftF, BASE_SPEED);
      analogWrite(RightF, BASE_SPEED);
    }
  }

  delay(30); // Loop timing
}

// Function to calculate distance in mm
long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  // Read pulse, with a 30ms timeout (approx 5 meters max)
  long duration = pulseIn(echo, HIGH, 30000); 
  
  if (duration == 0) return 999; // Return high number if no echo (clear path)
  
  // Speed of sound = 0.343 mm/us
  long distance = (duration * 0.343) / 2.0; 
  return distance;
}
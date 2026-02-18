// --- Pin Definitions ---
// Motors
const int RightF = 3;  // Forward PWM
const int RightB = 9;  // Reverse PWM
const int LeftF = 10;  // Forward PWM
const int LeftB = 11;  // Reverse PWM

// Ultrasonic Sensor
const int trigPin = A1;
const int echoPin = A0;

// --- Variables ---
long duration;
int distance;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600); 
  
  // Set Pin Modes
  pinMode(RightF, OUTPUT);
  pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);
  pinMode(LeftB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

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
  // 1. Trigger the Ultrasonic Sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 2. Measure the Echo
  duration = pulseIn(echoPin, HIGH);

  // 3. Calculate Distance
  // Distance = (Time * Speed of Sound) / 2 (there and back)
  distance = duration * 0.034 / 2;

  // 4. Output to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // 5. Logic for Detection
  if (distance <= 30 && distance > 0) {
    Serial.println("--- Object Detected within 5cm! ---");
    // Optional: Add code here to stop the robot if it gets too close
    analogWrite(RightF, 20);
    analogWrite(LeftF, 20);
    if (distance <= 20)
    {
      analogWrite(RightF, 0);
      analogWrite(LeftF, 0);
    }
  }

  // Small delay to stabilize readings
  delay(200);
}
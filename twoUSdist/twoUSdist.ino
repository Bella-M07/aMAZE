// HC-SR04 Ultrasonic Sensor Pins
// Sensor 1
#define TRIG_PIN1 D5  // GPIO14
#define ECHO_PIN1 D6  // GPIO12

// Sensor 2
#define TRIG_PIN2 D7  // GPIO13
#define ECHO_PIN2 D8  // GPIO15

void setup() {
  Serial.begin(9600);
  
  // Initialize sensor 1 pins
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  
  // Initialize sensor 2 pins
  
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  
  Serial.println("HC-SR04 Dual Sensor Ready!");
}

void loop() {
  // Measure distance from sensor 1
  long distance1 = measureDistance(TRIG_PIN1, ECHO_PIN1);
  
  // Small delay between sensor readings
  delay(50);
  
  // Measure distance from sensor 2
  long distance2 = measureDistance(TRIG_PIN2, ECHO_PIN2);
  
  // Display results
  Serial.print("Sensor 1: ");
  if (distance1 > 0 && distance1 < 300) {
    Serial.print(distance1);
    Serial.print(" cm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" | Sensor 2: ");
  if (distance2 > 0 && distance2 < 300) {
    Serial.print(distance2);
    Serial.println(" cm");
  } else {
    Serial.println("Out of range");
  }
  
  delay(100);
}

long measureDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10us pulse to trigger
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin, timeout after 30ms
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  // Calculate distance in cm
  long distance = duration * 0.0343 / 2;
  
  return distance;
}
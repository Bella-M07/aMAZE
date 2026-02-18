// Define pins
const int trigPin = A1;
const int echoPin = A0;

// Define variables
long duration;
int distance;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600); 
  
  // Set pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // 1. Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 2. Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. Read the echoPin (returns sound wave travel time in microseconds)
  duration = pulseIn(echoPin, HIGH);

  // 4. Calculate the distance
  // Speed of sound is 0.034 cm/us
  // Distance = (Time x Speed) / 2
  distance = duration * 0.034 / 2;
  Serial.println(distance);

  // 5. Check if object is within 7 cm
  // We use > 0 to filter out potential sensor errors reading 0
  if (distance <= 5 && distance > 0) {
    Serial.println("Detected");
  } else {
    // Optional: Print actual distance for debugging
    // Serial.print("Distance: ");
    // Serial.println(distance);
  }

  // Small delay to prevent spamming the serial monitor too fast
  delay(200);
}
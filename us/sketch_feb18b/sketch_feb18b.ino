// Define Pins
const int trig1 = A0; const int echo1 = A1; //right
const int trig2 = A2; const int echo2 = A3; //center
const int trig3 = A4; const int echo3 = A5; //left

void setup() {
  Serial.begin(9600);
  
  // Set all Trig pins as OUTPUT and Echo pins as INPUT
  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT); pinMode(echo3, INPUT);
}

void loop() {
  long dist1 = getDistance(trig1, echo1);
  delay(20); // Short delay to prevent sound interference
  long dist2 = getDistance(trig2, echo2);
  delay(20);
  long dist3 = getDistance(trig3, echo3);
  delay(20);

  // Print results
  Serial.print("L: "); Serial.print(dist1);
  Serial.print("cm | C: "); Serial.print(dist2);
  Serial.print("cm | R: "); Serial.println(dist3);
  
  delay(100); // Wait a bit before next sweep
}

// Function to handle the math
long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  long duration = pulseIn(echo, HIGH);
  long distance = duration * 0.034 / 2; // Speed of sound math
  return distance;
}
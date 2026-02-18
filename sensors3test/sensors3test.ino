// Define Pins
const int trig1 = A5; const int echo1 = A4; //left
const int trig2 = A1; const int echo2 = A0; //center
const int trig3 = A3; const int echo3 = A2; //Right

//define motor driver setup 

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
  Serial.print("mm | C: "); Serial.print(dist2);
  Serial.print("mm | R: "); Serial.println(dist3);
  
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
  long distance = duration * 0.343 / 2.0; // Speed of sound math
  return distance;
}
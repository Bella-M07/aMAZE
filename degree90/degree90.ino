const int RightF = 3;  const int RightB = 9;
const int LeftF = 10; const int LeftB = 11;

void setup() {
  // put your setup code here, to run once:
  pinMode(RightF, OUTPUT); pinMode(RightB, OUTPUT);
  pinMode(LeftF, OUTPUT);  pinMode(LeftB, OUTPUT);

  analogWrite(LeftF, 255);
  analogWrite(RightB,-10);
  // analogWrite(LeftB, 0);
  // analogWrite(RightF, 0);/
  delay(150);

  analogWrite(LeftF, 0);
  analogWrite(RightB, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}

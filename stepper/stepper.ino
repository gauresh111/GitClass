#define dirPin 22
#define stepPin 7
#define stepsPerRevolution 6400 //425
void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, HIGH);
}
void loop() {
  // These four lines result in 1 step:
  // Set the spinning direction clockwise:
  digitalWrite(dirPin, HIGH);

  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < stepsPerRevolution/4; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10);
  }

  delay(50);
;
  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);

  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < stepsPerRevolution/4; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(10);
  }

  delay(50);
}
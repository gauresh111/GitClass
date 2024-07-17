#include <PS4Controller.h>
int analog_picking = 17;
int digital_picking = 16;
void pinouts() {
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // pinMode(analog_shooter, INPUT);
  pinMode(analog_picking, OUTPUT);
  // pinMode(analog_actuator, INPUT);

  // pinMode(digital_shooter, OUTPUT);
  pinMode(digital_picking, OUTPUT);
  // pinMode(digital_actuator, OUTPUT);
  delay(100);
  analogWrite(analog_picking, 0);
  digitalWrite(digital_picking, HIGH);
  // pinMode(digital_actuator, OUTPUT);
  delay(100);

  pinouts();
  PS4.begin("7c:9e:bd:47:94:09");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (PS4.Up() || PS4.Down())
    if (PS4.Up()) {
      analogWrite(analog_picking, 250);
      digitalWrite(digital_picking, HIGH);
    }
  if (PS4.Down()) {
    analogWrite(analog_picking, 250);
    digitalWrite(digital_picking, LOW);
  }

  else {
    analogWrite(analog_picking, 0);
    digitalWrite(digital_picking, HIGH);
  }
}
// // void move_shooter(int direction) {
// //   /analogWrite(analog_shooter, 250);
// //   digitalWrite(digital_picking, direction);
// // }
// void move_picking(int direction) {
//   analogWrite(analog_picking, 250);
//   digitalWrite(digital_picking, direction);
// }
// // void stop_shooter() {
// //   analogWrite(analog_shooter, 0);
// //   digitalWrite(digital_picking, 0);
// //   //Serial.println("stop_shooter");
// //}
// void stop_picking() {
//   analogWrite(analog_picking, 0);
//   digitalWrite(digital_picking, 0);
//   //Serial.println("stop_picking");
// }
// void move_actuator(int direction) {
//   analogWrite(analog_actuator, 250);
//   digitalWrite(digital_picking, direction);
// }
// void stop_actuator() {
//   analogWrite(analog_actuator, 0);
//   digitalWrite(digital_picking, 0);
// }

int colour = 4;
void printcolour() {
  if (refre > 25 && refre < 77) {
    Serial.print("red");
    colour = 1;  // red

  } else if() {
    Serial.print("blue");
    colour = 2;
  } else if() {
    Serial.print("gree");
    colour = 3;
  }
  else{
    colour = 4;
  }
  move_servo(colour);
}

void move_servo(int colour)
{
if(colour==1)
{
  Servo1.write(45);
  delay(100);
}

}
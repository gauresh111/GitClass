#include <ESP32Servo.h>
#include <ezButton.h>
#include <PS4Controller.h>
Servo servo_R1;
Servo servo_L1;
Servo servo_R2;
Servo servo_L2;
ezButton LSwitch_L1(18);
ezButton LSwitch_L2(22);  //34
ezButton LSwitch_R1(21);  //35
ezButton LSwitch_R2(33);
int position = 0;
int pos_R1 = 0;
int pos_L1 = 180;
int pos_R2 = 0;
int pos_L2 = 180;
int anL = 17;   //AN1 yellow
int digL = 5;   //IN1 white
int anR = 4;    //AN2 red
int digR = 16;  //IN2 orange
int L_PWM = 255;
int R_PWM = 255;
int Strafe_PWM = 150;
int anStrafe = 13;
int digStrafe = 14;
int servo1 = 85;
int servo2 = 90;
void setupPinMode() {
  pinMode(anL, INPUT);
  pinMode(digL, OUTPUT);
  pinMode(anR, INPUT);
  pinMode(digR, OUTPUT);
  pinMode(anStrafe, INPUT);
  pinMode(digStrafe, OUTPUT);
}
void controller_event_cb(ps4_t ps4, ps4_event_t event) {
}

void climbingMechLeft_UP() {
  //Serial.println("Left Up ----------");
  analogWrite(anL, L_PWM);
  digitalWrite(digL, LOW);
}

void climbingMechLeft_DOWN() {
  //Serial.println("Left Down ----------");
  analogWrite(anL, L_PWM);
  digitalWrite(digL, HIGH);
}

void climbingMechLeft_STOP() {
  //Serial.println("Left Stop ----------");
  analogWrite(anL, 0);
  digitalWrite(digL, HIGH);
}

void climbingMechRight_UP() {
  // Serial.println("Right Up ----------");
  analogWrite(anR, R_PWM);
  digitalWrite(digR, HIGH);
}

void climbingMechRight_DOWN() {
  //Serial.println("Right Down ----------");
  analogWrite(anR, R_PWM);
  digitalWrite(digR, LOW);
}

void climbingMechRight_STOP() {
  //Serial.println("Right Stop ----------");
  analogWrite(anR, 0);
  digitalWrite(digR, HIGH);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupPinMode();
  // put your setup code here, to run once:
  ps4SetEventCallback(controller_event_cb);
  PS4.begin("B8:D6:1A:67:70:22");  //RR PermBM: B8:D6:1A:67:70:22
  servo_L1.attach(25);
  servo_L2.attach(32);
  servo_R1.attach(23);
  servo_R2.attach(19);
  setupPinMode();
  Servo_initial();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(servo2);
  servo_R1.write(0);
  delay(1000);
  servo_R1.write(90);
  delay(1000)
}

void moveServo_R1(int pos) {
  int offset = 60;  //53
  pos_R1 = pos + offset;
  servo_R1.write(pos_R1);
}

void moveServo_L1(int pos) {

  int offset = 50;  //46
  pos_L1 = 180 - pos - offset;
  servo_L1.write(pos_L1);
}

void moveServo_R2(int pos) {
  int offset = 11;  //11
  pos_R2 = pos + offset;
  servo_R2.write(pos_R2);
}

void moveServo_L2(int pos) {

  int offset = 0;  //2
  pos_L2 = 180 - pos - offset;
  servo_L2.write(pos_L2);
}
void Servo_initial() {
  for (int i = 0; i <= 85; i++) {
    moveServo_R1(i);
    moveServo_L1(i);
    Serial.print("Part 1: ");
    Serial.println(i);
    delay(15);
  }
  for (int i = 0; i <= 90; i++) {
    moveServo_R2(i);
    moveServo_L2(i);
    Serial.print("Part 3: ");
    Serial.println(i);
    delay(15);
  }
}
void climbingMechStrafe_RIGHT() {
  //Serial.println("Strafing ----------");
  analogWrite(anStrafe, Strafe_PWM);
  digitalWrite(digStrafe, LOW);
}
void climbingMechStrafe_LEFT() {
  // Serial.println("Strafing ----------");
  analogWrite(anStrafe, Strafe_PWM);
  digitalWrite(digStrafe, HIGH);
}

void climbingMechStrafe_STOP() {
  analogWrite(anStrafe, 0);
  digitalWrite(digStrafe, HIGH);
}

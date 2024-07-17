#include <PS4Controller.h>

int arrayOfPins[] = { 26, 36, 25, 39, 33, 34, 32, 35 };
//motor
int ir1 = 26;
int ir2 = 16;
int enableB = 17;
int ir4 = 5;
int ir3 = 18;
int enableA = 25;
int PWM = 100;


//Bluetooth


void setup_pinMODE() {
  pinMode(enableB, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(ir4, OUTPUT);
  pinMode(ir3, OUTPUT);
  pinMode(ir2, OUTPUT);
  pinMode(ir1, OUTPUT);
  for (int i = 0; i < 8; i++) {
    pinMode(arrayOfPins[i], INPUT);
  }
  stop();
}
void forward(int left, int right) {
  analogWrite(enableA, left);
  analogWrite(enableB, right);
  digitalWrite(ir4, HIGH);
  digitalWrite(ir3, LOW);
  digitalWrite(ir2, HIGH);
  digitalWrite(ir1, LOW);
}
void backward(int left, int right) {
  analogWrite(enableA, left);
  analogWrite(enableB, right);
  digitalWrite(ir4, LOW);
  digitalWrite(ir3, HIGH);
  digitalWrite(ir2, LOW);
  digitalWrite(ir1, HIGH);
}
void stop() {
  analogWrite(enableA, 0);
  analogWrite(enableB, 0);
  digitalWrite(ir4, HIGH);
  digitalWrite(ir3, HIGH);
  digitalWrite(ir2, HIGH);
  digitalWrite(ir1, HIGH);
}
void left() {
  analogWrite(enableA, left);
  analogWrite(enableB, right);
  digitalWrite(ir4, HIGH);
  digitalWrite(ir3, LOW);
  digitalWrite(ir2, LOW);
  digitalWrite(ir1, HIGH);
}
void right() {
  analogWrite(enableA, left);
  analogWrite(enableB, right);
  digitalWrite(ir4, LOW);
  digitalWrite(ir3, HIGH);
  digitalWrite(ir2, HIGH);
  digitalWrite(ir1, LOW);
}
void clockwise() {
  analogWrite(enableA, 5);
  analogWrite(enableB, 255);
  digitalWrite(ir4, HIGH);
  digitalWrite(ir3, LOW);
  digitalWrite(ir2, HIGH);
  digitalWrite(ir1, LOW);
}
id setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  setup_pinMODE();
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  forward();
}

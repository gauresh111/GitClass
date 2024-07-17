

#include "Wire.h"



#define I2C_DEV_ADDR 0x55

int roller_digital = 26;
int roller_analog = 14;
int lift_digital = 27;
int lift_analog = 13;
int relay_pin = 4;



void onReceive(int len) {
  // Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    int value = Wire.read();
    if (value == 50) {
      move_roller(75, 0);
    } else if (value == 49) {
      move_roller(75, 1);
    } else if (value == 51) {
      move_roller(0, 0);
    }
    if (value == 52) {
      move_lift(240, 0);
    } else if (value == 54) {
      move_lift(240, 1);
    } else if (value == 55) {
      move_lift(0, 0);
    }
    if (value == 53) {
      digitalWrite(relay_pin, 0);
      delay(200);
      digitalWrite(relay_pin, 1);
    }

    Serial.println(value);
  }
  Serial.println();
}


void setup() {
  // put your setup code here, to run once:
  pinMode(lift_analog, INPUT);
  pinMode(lift_digital, OUTPUT);
  pinMode(roller_analog, INPUT);
  pinMode(roller_digital, OUTPUT);
  Serial.begin(9600);
  move_lift(0, 1);
  move_roller(0, 1);
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, 1);
  Wire.onReceive(onReceive);
  //Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

  delay(100);
}

void loop() {
}

void move_roller(int speed, int direction) {

  analogWrite(roller_analog, speed);
  digitalWrite(roller_digital, direction);
}
void move_lift(int speed, int direction) {
  analogWrite(lift_analog, speed);
  digitalWrite(lift_digital, direction);
}
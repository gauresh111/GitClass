#include "Wire.h"
#include <PS4Controller.h>
#include <ezButton.h>
#include <ESP32Servo.h>


#define I2C_DEV_ADDR 0x55
int Setpoint=100;
ezButton LSwitch_shooting_max(36);
ezButton LSwitch_shooting_min(39);
ezButton LSwitch_LIFT_up_max(35);    //34
ezButton LSwitch_LIFT_down_min(34);  //35


int roller_digital = 17;
int roller_analog = 18;
int lift_digital = 5;
int lift_analog = 19;
int relay_pin = 26;

int LSwitch_shooting_Max = 0;
int LSwitch_shooting_Min = 0;
int LSwitch_LIFT_up_Max = 0;
int LSwitch_LIFT_down_Min = 0;
bool check_servo = false;

Servo servo_R1;
void onRequest() {

  Wire.print(" Packets.");
  Serial.println("onRequest");
}

void onReceive(int len) {
  // Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    int value = Wire.read();
    if (value == 50) {
      digitalWrite(relay_pin, LOW);
    } else if (value == 49) {
      digitalWrite(relay_pin, HIGH);
    }
    Serial.println(value);
  }
  Serial.println();
}


void onConnect() {
  Serial.println("Connected");
}

void controller_event_cb(ps4_t ps4, ps4_event_t event) {
  if (event.button_down.square) {
    servo_R1.write(30);
    delay(100);
    servo_R1.write(75);

    // if (check_servo == false) {
    //   servo_R1.write(75);
    //   check_servo = true;
    // } else {
    //   check_servo = false;
    //   servo_R1.write(30);
    // }
  }
  if (event.button_down.up) {
    
    if (Setpoint <= 260 || Setpoint >=255 ) {
      if(Setpoint >=240)
      {
        Setpoint = 255;  
      }
      else{
           Setpoint = Setpoint + 20;
      }
      Serial.println("-- Setpoint Incremented--");
     
    }
  }
  if (event.button_down.down) {
    if (Setpoint > 50) {
      Serial.println("-- Setpoint Decremented--");
      Setpoint = Setpoint - 20;
    }
  }

}

void setup() {
  // put your setup code here, to run once:
  PS4.attachOnConnect(onConnect);
  PS4.begin("7c:9e:bd:47:94:06");  // 7c:9e:bd:47:94:06 joel // b8:d6:1a:67:70:20 nathan
  ps4SetEventCallback(controller_event_cb);
  pinMode(lift_analog, INPUT);
  pinMode(lift_digital, OUTPUT);
  pinMode(roller_analog, INPUT);
  pinMode(roller_digital, OUTPUT);
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  move_lift(0, 1);
  move_roller(0, 1);
  servo_R1.attach(4);
  delay(100);
  pinMode(relay_pin, OUTPUT);
  servo_R1.write(75);
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (PS4.L1()) {
    move_lift(200, 0);
    Serial.println("lift _ down");
  } else if (PS4.R1()) {
    move_lift(200, 1); 
    Serial.println("lift _ up");
  } else {
    move_lift(0, 1);
    // Serial.println("stop lift");
    // move_roller(0, 1);
  }
  if (PS4.R2()) {
    move_roller(Setpoint, 1);
    Serial.println("roller _ up");
  } else if (PS4.L2()) {
    move_roller(Setpoint, 0);
    Serial.println("roller _ down");
  } else {
    // move_lift(0, 1);
    move_roller(0, 1);
    //Serial.println("stop roller");
  }

  get_limitSwitch();
  Serial.println(Setpoint);
}
void move_roller(int speed, int direction) {

  analogWrite(roller_analog, speed);
  digitalWrite(roller_digital, direction);
}
void move_lift(int speed, int direction) {
  analogWrite(lift_analog, speed);
  digitalWrite(lift_digital, direction);
}
void get_limitSwitch() {
  LSwitch_shooting_max.loop();
  LSwitch_shooting_min.loop();
  LSwitch_LIFT_up_max.loop();
  LSwitch_LIFT_down_min.loop();
  
  LSwitch_shooting_Max = LSwitch_shooting_max.getState();
  LSwitch_shooting_Min = LSwitch_shooting_min.getState();
  LSwitch_LIFT_up_Max = LSwitch_LIFT_up_max.getState();
  LSwitch_LIFT_down_Min = LSwitch_LIFT_down_min.getState();
  // Serial.print("shooting max value: ");
  // Serial.print( nb LSwitch_shooting_Max);
  // Serial.print("  shooting min value: ");
  // Serial.print(LSwitch_shooting_Min);
  // Serial.print("  LIFT max value: ");
  // Serial.print(LSwitch_LIFT_up_Max);
  // Serial.print("  LIFT min value: ");
  // Serial.println(LSwitch_LIFT_down_Min);
}
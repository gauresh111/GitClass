#include <PS4Controller.h>
#include "Arduino.h"
#include <ESP32Servo.h>


int RPM = 150;
int boost_RPM = 250;
int isLocked = 0;
int sensitivity = 40;
int turn;

Servo steering_wheel;

int r = 255;
int g = 0;
int b = 0;

// l298n pinouts
const int ena = 5;
const int enb = 5;
const int in1 = 16;
const int in2 = 17;
const int in3 = 18;
const int in4 = 19;

// Calculates the next value in a rainbow sequence
void nextRainbowColor() {
  if (r > 0 && b == 0) {
    r--;
    g++;
  }
  if (g > 0 && r == 0) {
    g--;
    b++;
  }
  if (b > 0 && g == 0) {
    r++;
    b--;
  }
}

//rgb value send

void led_send(void* pvParameters) {
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (isLocked == false) {
      if (PS4.isConnected()) {
        PS4.setLed(r, g, b);
        nextRainbowColor();
        //PS4.setFlashRate(1000, 200);
        PS4.sendToController();
        delay(10);
        //        Serial.print("direction :");
        // Serial.println(turn);
      }
    }
    Serial.println(isLocked);
  }
}

void onConnect() {
  Serial.println("Connected");
  PS4.setLed(50, 205, 50);
  PS4.sendToController();
}

// ps4 call black function
void controller_event_cb(ps4_t ps4, ps4_event_t event) {
  if (event.button_down.square) {
    if (!isLocked) {
      Serial.println("-- Controller Locked --");
      PS4.setLed(255, 0, 0);
      PS4.setFlashRate(0, 0);
      PS4.sendToController();
      isLocked = 1;
    } else {
      Serial.println("-- Controller Unlocked --");

      isLocked = 0;
    }
  }
}

//direction function
void forward(int rpm) {
  int rxAxis = PS4.RStickX();
  int ryAxis = PS4.RStickY() * (1);
  int mlxAxis = RPM;
  int mlyAxis = RPM;
  int mrxAxis = RPM;
  int mryAxis = RPM;
  Serial.println("FOrward");
  //turning
  if (rxAxis > sensitivity && abs(ryAxis) < 50) {
    turn = map(abs(rxAxis), 40, 128, 90, 0);
    strafe(turn, mlxAxis);
  } else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {

    turn = map(abs(rxAxis), 40, 128, 90, 180);
    strafe(turn, mlxAxis);
  } else {
    turning_center();
  }
  analogWrite(ena, rpm);
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);

  //*/ end of turning
}
void reverse(int rpm) {
  int rxAxis = PS4.RStickX();
  int ryAxis = PS4.RStickY() * (1);
  int mlxAxis = RPM;
  int mlyAxis = RPM;
  int mrxAxis = RPM;
  int mryAxis = RPM;
  Serial.println("backward");
  //turning
  if (rxAxis > sensitivity && abs(ryAxis) < 50) {
    turn = map(abs(rxAxis), 40, 128, 90, 0);
    strafe(turn, mlxAxis);
  } else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {

    turn = map(abs(rxAxis), 40, 128, 90, 180);
    strafe(turn, mlxAxis);
  } else {
    turning_center();
  }
  //*/ end of turning
  analogWrite(ena, rpm);
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
}
void strafe(int direction, int rpm) {
  Serial.print("direction :");
  Serial.println(direction);
  steering_wheel.write(direction);
}
void stop() {
  turning_center();
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}


void turning_center() {
  turn = 90;
  // Serial.print("direction :");
  // Serial.println(turn);
  steering_wheel.write(90);
}
//locmotion function
void Locomote() {
  int lxAxis = PS4.LStickX();
  int lyAxis = PS4.LStickY() * (1);
  int rxAxis = PS4.RStickX();
  int ryAxis = PS4.RStickY() * (1);

  int mlxAxis = RPM;
  int mlyAxis = RPM;
  int mrxAxis = RPM;
  int mryAxis = RPM;
  //getRPM();

  if (lyAxis < -sensitivity && abs(lxAxis) < 50) {

    reverse(mlyAxis);
  }

  else if (lyAxis > sensitivity && abs(lxAxis) < 50) {


    forward(mlyAxis);
  }

  else if (rxAxis > sensitivity && abs(ryAxis) < 50) {
    turn = map(abs(rxAxis), 40, 128, 90, 0);
    strafe(turn, mlxAxis);
  }

  else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {

    turn = map(abs(rxAxis), 40, 128, 90, 180);
    strafe(turn, mlxAxis);
  } else {
    stop();
  }
}
//intializing pins
void set_l298n_pins() {
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}
void setup() {
  // put your setup code here, to run once:
  /* pin task to core 0 */
  Serial.begin(9600);
  set_l298n_pins();
  xTaskCreatePinnedToCore(led_send, "Task1", 10000, NULL, 0, NULL, 0);
  ps4SetEventCallback(controller_event_cb);
  PS4.attachOnConnect(onConnect);
  PS4.begin("b8:d6:1a:67:70:22");
  steering_wheel.attach(4);
  delay(100);
  stop();
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (isLocked == false) {
    Locomote();
  }
}

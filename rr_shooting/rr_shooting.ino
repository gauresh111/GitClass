#include <ezButton.h>
#include <PS4Controller.h>

ezButton start_switch(19);
ezButton Brake_switch(23);
int en1 = 16;
int an1 = 4;
int ENCODER_A = 15;  // Pin for Encoder A
int ENCODER_B = 5;
int target = 1;
volatile int encoderValueSet = 0;
void getEncoderCounter() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet--;
  } else {
    encoderValueSet++;
  }
}
void controller_event_cb(ps4_t ps4, ps4_event_t event) {

  if (event.button_down.up) {
    if (target > 0 && target < 4) {
      target = target + 1;
    }
  }
  if (event.button_down.down) {
    if (target > 0 && target < 4) {
      target = target - 1;
    }
  }
}
void onConnect() {
  Serial.println("Connected");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(en1, OUTPUT);
  pinMode(an1, INPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), getEncoderCounter, CHANGE);
  ps4SetEventCallback(controller_event_cb);
  PS4.attachOnConnect(onConnect);
  PS4.begin("d4:d4:da:5c:2f:86");
  goto_initiall();
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  reload_switch();
  Target_pipe(target);
}
void goto_initiall() {
  while (start_switch.getState() == 0) {
    analogWrite(an1, 100);
    digitalWrite(en1, LOW);
    Serial.println("MOVing motor to start position");
    reload_switch();

  }
  Serial.println("ready");
  encoderValueSet = 0;
}
void reload_switch() {
  Brake_switch.loop();
  start_switch.loop();
}
void Target_pipe(int target) {
  int direction = 0;
  switch (target) {
    case 1:
      {
        //set launcher for pipe 1
        Serial.println("pipe 1");
        //encoderValueSet==1000;
        if (encoderValueSet > 1000) {
          direction = 0;
        } else {
          direction = 1;  //forward
        }
        while (encoderValueSet < 980 || encoderValueSet > 1020) {
          analogWrite(an1, 100);
          digitalWrite(en1, direction);
         // Serial.println("hello");
        }
         Serial.print(direction);
        break;
      }

    case 2:
      {
        //set launcher for pipe 1
        Serial.println("pipe 2");
        //encoderValueSet==1000;
        if (encoderValueSet > 2000) {
          direction = 0;
        } else {
          direction = 1;  //forward
        }
        while (encoderValueSet < 1980 || encoderValueSet > 2020) {
          analogWrite(an1, 100);
          digitalWrite(en1, direction);
        }
        break;
      }
    case 3:
      {
        //set launcher for pipe 1
        Serial.println("pipe 3");
        //encoderValueSet==1000;
        if (encoderValueSet > 3000) {
          direction = 0;
        } else {
          direction = 1;  //forward
        }
        while (encoderValueSet < 2980 || encoderValueSet > 3020) {
          analogWrite(an1, 100);
          digitalWrite(en1, direction);
        }
        break;
      }
  }
}
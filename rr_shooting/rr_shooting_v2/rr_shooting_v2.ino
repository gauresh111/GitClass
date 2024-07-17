#include <ezButton.h>
#include <PS4Controller.h>

ezButton start_switch(21);
ezButton Brake_switch(22);
int en1 = 5;
int an1 = 17;
int ENCODER_A = 36;  // Pin for Encoder A
int ENCODER_B = 39;
int target = 1;
int r = 255;
int g = 0;
int b = 0;

int shooting_relay = 25;
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
    if (target >= 1 && target < 3) {
      target = target + 1;
      Serial.println(target);
      sendcolour(target);
    }
  }
  if (event.button_down.down) {
    if (target > 1 && target <= 3) {
      target = target - 1;
      Serial.println(target);
      sendcolour(target);
    }
  }
}
void sendcolour(int target) {
  if (target == 1) {
    PS4.setLed(169, 69, 69);
  } else if (target == 2) {
    PS4.setLed(69, 169, 69);
  } else if (target == 3) {
    PS4.setLed(69, 69, 169);
  }

  PS4.sendToController();
  delay(10);
}
void onConnect() {
  Serial.println("Connected");
  Serial.print(PS4.Battery());
  sendcolour(target);
  delay(1000);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(en1, OUTPUT);
  pinMode(an1, INPUT);
  pinMode(shooting_relay, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), getEncoderCounter, RISING);
  ps4SetEventCallback(controller_event_cb);
  PS4.attachOnConnect(onConnect);
  PS4.begin("7c:9e:bd:47:94:09");
  delay(2000);
 // goto_initiall();
}


void loop() {
  // put your main code here, to run repeatedly:
  reload_switch();
 
  if (PS4.Circle()) {
    digitalWrite(shooting_relay, HIGH);
  }
  if (PS4.Square()) {
    digitalWrite(shooting_relay, LOW);
  }
  if (PS4.L2() || PS4.R2()) {
    if (PS4.L2()) {
      motorStart(1);
      Serial.print("L2");
    }
    if (PS4.R2()) {
      motorStart(0);
      Serial.print("R2");
    }
  } else {  //13450 21150
    motorStop();
  }
 

  //Serial.println(encoderValueSet);
}
void goto_initiall() {
  while (start_switch.getState() == 0) {
    analogWrite(an1, 100);
    digitalWrite(en1, LOW);
    Serial.println("Moving motor to start position");
    reload_switch();
  }
  Serial.println("ready");
  motorStop();
  delay(1500);
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
        while (abs(encoderValueSet) < 900 || abs(encoderValueSet) > 1100) {

          motorStart(direction);
          Serial.println(encoderValueSet);
          Serial.println("moving pipe1");
          if (Brake_switch.getState() == 1) {
            motorStop();


            Serial.print("break");
            reload_switch();

            break;
          }
          reload_switch();
        }
        motorStop();
        //Serial.print(direction);

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
        while (abs(encoderValueSet) < 1900 || abs(encoderValueSet) > 2100) {
          motorStart(direction);


          Serial.println("moving pipe2");
          if (Brake_switch.getState() == 1) {


            motorStop();
            Serial.print("break");
            reload_switch();

            break;
          }
          reload_switch();
        }
        motorStop();
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
        while (abs(encoderValueSet) < 2900 || abs(encoderValueSet) > 3100) {
          motorStart(direction);

          Serial.println("moving pipe3");
          if (Brake_switch.getState() == 1) {


            motorStop();
            Serial.print("break");
            reload_switch();
            break;
          }
          reload_switch();
        }
        motorStop();
        break;
      }
  }
}

void motorStart(int direction) {
  analogWrite(an1, 255);
  digitalWrite(en1, direction);
}

void motorStop() {
  analogWrite(an1, 0);
  digitalWrite(en1, HIGH);
}
void automatic() {
  goto_initiall();
  digitalWrite(shooting_relay, HIGH);
  delay(2000);
  digitalWrite(shooting_relay, LOW);
   delay(1000);
  encoderValueSet = 0;
  while (abs(encoderValueSet) < 21150) {
    motorStart(1);
    Serial.println("in");
  }
    Serial.println(encoderValueSet);
//  digitalWrite(shooting_relay, HIGH);
  goto_initiall();
}

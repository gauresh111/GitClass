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

int anL = 17;   //AN1 yellow
int digL = 5;   //IN1 white
int anR = 4;    //AN2 red
int digR = 16;  //IN2 orange
int anStrafe = 13;
int digStrafe = 14;

int position = 0;
int pos_R1 = 0;
int pos_L1 = 180;
int pos_R2 = 0;
int pos_L2 = 180;

int L_PWM = 150;
int R_PWM = 255;
int Strafe_PWM = 150;

int LS_L1_State = 0;
int LS_L2_State = 0;
int LS_R1_State = 0;
int LS_R2_State = 0;

int isMechRunning = 0;

void onConnect() {
  Serial.println("Connected");
}

void onDisConnect() {
  Serial.println("Disconnected");
}

void controller_event_cb(ps4_t ps4, ps4_event_t event) {
  if (event.button_down.square) {
    if (!isMechRunning) {
      isMechRunning = 1;
      Serial.println("-- Start Climbing Mechanism --");
      climbingMech();
      isMechRunning = 0;
    } else {
      Serial.println("-- Already Running --");
    }
  }
}

void setupPinMode() {
  pinMode(anL, INPUT);
  pinMode(digL, OUTPUT);
  pinMode(anR, INPUT);
  pinMode(digR, OUTPUT);
  pinMode(anStrafe, INPUT);
  pinMode(digStrafe, OUTPUT);
}


void setup() {
  delay(2000);
  Serial.begin(9600);
  setupPinMode();
  // put your setup code here, to run once:
  ps4SetEventCallback(controller_event_cb);
  PS4.attachOnConnect(onConnect);
  PS4.begin("B8:D6:1A:67:70:22");  //RR PermBM: B8:D6:1A:67:70:22
  servo_L1.attach(25);
  servo_L2.attach(32);
  servo_R1.attach(23);
  servo_R2.attach(19);

  Serial.println("Servo set Initial Started");
  Servo_initial();
  Serial.println("Servo set Initial Finished");

  Serial.println("Ready");
}
void loop() {
  LSwitch_L1.loop();
  LSwitch_L2.loop();
  LSwitch_R1.loop();
  LSwitch_R2.loop();
}

void Servo_initial() {
  for (int i = 0; i <= 90; i++) {
    moveServo_R1(i);
    moveServo_L1(i);
    Serial.print("Part 1: ");
    Serial.println(i);
    delay(15);
  }
  for (int i = 90; i >= 0; i--) {
    moveServo_R2(i);
    moveServo_L2(i);
    Serial.print("Part 3: ");
    Serial.println(i);
    delay(15);
  }
  for (int i = 90; i >= 0; i--) {
    moveServo_R1(i);
    moveServo_L1(i);
    Serial.print("Part 4: ");
    Serial.println(i);
    delay(15);
  }
  delay(1000);
}

void climbingMech_Servo(int config) {
  //config 1 = Open
  //config 0 = Close

  if (config == 1) {
    for (int i = 0; i <= 90; i++) {
      moveServo_R1(i);
      moveServo_L1(i);
      Serial.print("Part 1: ");
      Serial.println(i);
      delay(10);
    }

    delay(1000);
    for (int i = 0; i <= 90; i++) {
      moveServo_R2(i);
      moveServo_L2(i);
      Serial.print("Part 2: ");
      Serial.println(i);
      delay(10);
    }

    delay(1000);
  } else {

    for (int i = 90; i >= 0; i--) {
      moveServo_R2(i);
      moveServo_L2(i);
      Serial.print("Part 3: ");
      Serial.println(i);
      delay(10);
    }
    delay(1000);

    for (int i = 90; i >= 0; i--) {
      moveServo_R1(i);
      moveServo_L1(i);
      Serial.print("Part 4: ");
      Serial.println(i);
      delay(10);
    }

    delay(1000);
  }
}


void climbingMech() {
  delay(1000);
  while (LSwitch_L1.getState() == false) {
    // Serial.println("Climbing Mech Left UP");
    climbingMechLeft_UP();
  }
  climbingMechLeft_STOP();
  Serial.println("Climbing Mech Left Stopped");

  Serial.println("Open Servo Set");
  delay(1000);
  climbingMech_Servo(1);
  delay(1000);

  while (LSwitch_L2.getState() == false || LSwitch_R2.getState() == false) {
    // Serial.println("Climbing Mech Down");
    if (LSwitch_L2.getState() == false) {
      climbingMechLeft_DOWN();
    } else {
      climbingMechLeft_STOP();
    }
    if (LSwitch_R2.getState() == false) {
      climbingMechRight_DOWN();
    } else {
      climbingMechRight_STOP();
    }
  }

  climbingMechLeft_STOP();
  climbingMechRight_STOP();
  Serial.println("Climbing Mech Stopped");
  delay(1000);

  int startTime = millis();
  int endTime = startTime;
  while (endTime - startTime < 5000) {
    endTime = millis();
    climbingMechStrafe_START();
    // Serial.println("Mech Strafing");
  }
  Serial.println("Mech Strafing Stopped");
  climbingMechStrafe_STOP();
  delay(1000);

  while (LSwitch_R1.getState() == false) {
    climbingMechRight_UP();
  }

  climbingMechRight_STOP();
  Serial.println("Climbing Mech Right Stopped");
  delay(1000);

  while (LSwitch_L1.getState() == false) {
    Serial.println("Climbing Mech Left UP");
    climbingMechLeft_UP();
  }
  climbingMechLeft_STOP();
  Serial.println("Climbing Mech Left Stopped");
  delay(1000);
  Serial.println("Close Servo Set");
  climbingMech_Servo(0);
  Serial.println("### Mission Accomplished ###");
}

void climbingMechLeft_UP() {
  Serial.println("Left Up ----------");
  analogWrite(anL, L_PWM);
  digitalWrite(digL, HIGH);
}

void climbingMechLeft_DOWN() {
  Serial.println("Left Down ----------");
  analogWrite(anL, L_PWM);
  digitalWrite(digL, LOW);
}

void climbingMechLeft_STOP() {
  analogWrite(anL, 0);
  digitalWrite(digL, HIGH);
}

void climbingMechRight_UP() {
  Serial.println("Right Up ----------");
  analogWrite(anR, R_PWM);
  digitalWrite(digR, HIGH);
}

void climbingMechRight_DOWN() {
  Serial.println("Right Down ----------");
  analogWrite(anR, R_PWM);
  digitalWrite(digR, LOW);
}

void climbingMechRight_STOP() {
  Serial.println("Right Stop ----------");
  analogWrite(anR, 0);
  digitalWrite(digR, HIGH);
}

void climbingMechStrafe_START() {
  Serial.println("Strafing ----------");
  // analogWrite(anStrafe, Strafe_PWM);
  // digitalWrite(digStrafe, LOW);
}

void climbingMechStrafe_STOP() {
  analogWrite(anStrafe, 0);
  digitalWrite(digStrafe, HIGH);
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

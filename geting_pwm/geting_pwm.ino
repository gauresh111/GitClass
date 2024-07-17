#include <PID_v1.h>
#include <PS4Controller.h>
#include <Wire.h>

#define I2C_DEV_ADDR 0x55

#define I2C_DEV_ADDR_shooter 0x50


int ENCODER_R1_A = 25;  // Pin for Encoder A
int ENCODER_R1_B = 26;
int ENCODER_R2_A = 35;  // Pin for Encoder A
int ENCODER_R2_B = 34;
int ENCODER_L1_A = 33;  // Pin for Encoder A
int ENCODER_L1_B = 32;
int ENCODER_L2_A = 39;  // Pin for Encoder A
int ENCODER_L2_B = 36;

int anL1 = 27;
int digL1 = 23;
int anL2 = 13;
int digL2 = 14;
int anR2 = 17;
int digR2 = 5;
int anR1 = 18;  //13
int digR1 = 19;

long duration;
int distance;
int Target = 0;
int initt = 0;
int isShareON = false;

/* 
    Blink without Delay, example here: arduino.cc/en/Tutorial/BlinkWithoutDelay
*/

// constants won't change. Used here to set a pin number :
// the number of the LED pin

// Variables will change :


#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int measuree = 0;
int setpoint = 50;

volatile int encoderValueSet_R2 = 0;
float rpm_R2 = 0;
volatile int encoderValueSet_R1 = 0;
float rpm_R1 = 0;
volatile int encoderValueSet_L1 = 0;
float rpm_L1 = 0;
volatile int encoderValueSet_L2 = 0;
float rpm_L2 = 0;
int RPM = 0;
int sensitivity = 60;
int isLocked = 0;
int input;
int stop = 100;
int counter = 35;
int is_diagonal = 0;
double Setpoint, InputR2, OutputR2;
double InputR1, OutputR1;
double InputL1, OutputL1;
double InputL2, OutputL2;
bool shooter_checked = false;
bool actuator_checked = false;
bool shooter_stop = false;
bool acuator_stop = false;
int intverall = 0;
int target = 0;
int currentConfig[4] = { 0, 0, 0, 0 };
int option_button = 1;
void getEncoderCounter_R1() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_R1_A);
  int B = digitalRead(ENCODER_R1_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_R1--;
  } else {
    encoderValueSet_R1++;
  }
}
void getEncoderCounter_R2() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_R2_A);
  int B = digitalRead(ENCODER_R2_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_R2--;
  } else {
    encoderValueSet_R2++;
  }
}

void onConnect() {
  Serial.println("Connected");
  PS4.setLed(50, 205, 50);
  PS4.sendToController();
}

void onDisConnect() {
  Serial.println("Disconnected");
}


void getEncoderCounter_L1() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_L1_A);
  int B = digitalRead(ENCODER_L1_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_L1--;
  } else {
    encoderValueSet_L1++;
  }
}

void getEncoderCounter_L2() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_L2_A);
  int B = digitalRead(ENCODER_L2_B);

  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_L2--;
  } else {
    encoderValueSet_L2++;
  }
}
void loco(void* pvParameters) {

  Serial.print("locomotion running on core ");

  Serial.println(xPortGetCoreID());

  for (;;) {
    Serial.print("Target:");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print("R1_rpm:");
    Serial.print(abs(rpm_R1));
    Serial.print(",");
    Serial.print("R2_rpm:");
    Serial.print(abs(rpm_R2));
    Serial.print(",");
    Serial.print("L1_rpm:");
    Serial.print(abs(rpm_L1));
    Serial.print(",");
    Serial.print("L2_rpm:");
    Serial.println(abs(rpm_L2));
    if (isLocked == false) {
      Locomote();

    }


    else {
      Stop();
    }
    if (PS4.L1()) {
      if (shooter_checked == false) {
        Wire.beginTransmission(I2C_DEV_ADDR_shooter);

        Wire.print(1);
       // Serial.println(1);
        Wire.endTransmission();
        shooter_checked = true;
        shooter_stop = false;
      }
    } else if (PS4.R1()) {
      if (shooter_checked == false) {
        Wire.beginTransmission(I2C_DEV_ADDR_shooter);
        Wire.print(2);
       // Serial.println(2);
        Wire.endTransmission();
        shooter_checked = true;
        shooter_stop = false;
      }
    } else {
      //move_acuator(0, 1);
      if (shooter_stop == false) {
        Wire.beginTransmission(I2C_DEV_ADDR_shooter);
        Wire.print(3);
      //  Serial.println(3);
        Wire.endTransmission();
        shooter_stop = true;
        shooter_checked = false;
      }
    }
    if (PS4.L2()) {
      if (actuator_checked == false) {
        Wire.beginTransmission(I2C_DEV_ADDR_shooter);
        Wire.print(4);
      //  Serial.println(4);
        Wire.endTransmission();
        actuator_checked = true;
        acuator_stop = false;
      }
    } else if (PS4.R2()) {
      if (actuator_checked == false) {
        Wire.beginTransmission(I2C_DEV_ADDR_shooter);
        Wire.print(5);
     //   Serial.println(5);
        Wire.endTransmission();
        actuator_checked = true;
        acuator_stop = false;
      }
    } else {
      if (acuator_stop == false) {

        Wire.beginTransmission(I2C_DEV_ADDR_shooter);
        Wire.print(6);
      //  Serial.println(6);
        Wire.endTransmission();
        acuator_stop = true;
        actuator_checked = false;
      }
    }
  }
}


void getrpm(void* pvParameters) {

  Serial.print("getrpm running on core ");

  Serial.println(xPortGetCoreID());

  for (;;) {
    int startTime = millis();
    int endTime = startTime;

    while (endTime - startTime < 100) {
      endTime = millis();
      // getEncoderCounter();
    }
    int lineCount = 268;  //650 for 750 rpm motor else 267
    rpm_R2 = (encoderValueSet_R2 * 10 * 60) / lineCount;
    rpm_R1 = (encoderValueSet_R1 * 10 * 60) / lineCount;
    rpm_L1 = (encoderValueSet_L1 * 10 * 60) / lineCount;
    rpm_L2 = (encoderValueSet_L2 * 10 * 60) / lineCount;

    encoderValueSet_R2 = 0;
    encoderValueSet_R1 = 0;
    encoderValueSet_L1 = 0;
    encoderValueSet_L2 = 0;



    //delay(50);
    // delay(50);
  }
}


void controller_event_cb(ps4_t ps4, ps4_event_t event) {
  if (event.button_down.square) {
    if (!isLocked) {
      Serial.println("-- Controller Locked --");
      isLocked = 1;
      PS4.setLed(119, 136, 153);
      PS4.setRumble(0, 0);
      PS4.sendToController();
    } else {
      Serial.println("-- Controller Unlocked --");
      PS4.setLed(50, 205, 50);
      PS4.setRumble(0, 0);
      PS4.sendToController();
      isLocked = 0;
    }
  }

  if (event.button_down.up) {
    if (Setpoint < 250) {
      Serial.println("-- Setpoint Incremented--");
      Setpoint = Setpoint + 10;
    }
  }
  if (event.button_down.down) {
    if (Setpoint > 50) {
      Serial.println("-- Setpoint Decremented--");
      Setpoint = Setpoint - 10;
    }
  }

  if (event.button_down.cross) {
    Wire.beginTransmission(I2C_DEV_ADDR);
    Wire.print(1);
    Wire.endTransmission();
 //   Serial.println(1);
  }
  if (event.button_down.circle) {
    Wire.beginTransmission(I2C_DEV_ADDR);
    Wire.print(2);
    Wire.endTransmission();
   // Serial.println(2);
  }
  if (event.button_down.share) {


    Wire.beginTransmission(I2C_DEV_ADDR_shooter);
    Wire.print("a");
    Wire.endTransmission();
    //Serial.println("a");
  }

  if (event.button_down.options) {
    if (option_button == 1) {
      PS4.setLed(255, 0, 0);
      PS4.setRumble(0, 0);
      PS4.sendToController();
      Wire.beginTransmission(I2C_DEV_ADDR_shooter);
      Wire.print(7);
      target = 1;
      Wire.endTransmission();
     // Serial.println(7);
      option_button++;
    } else if (option_button == 2) {
      PS4.setLed(0, 255, 0);
      PS4.sendToController();
      Wire.beginTransmission(I2C_DEV_ADDR_shooter);
      Wire.print(8);
      Wire.endTransmission();
      //Serial.println(8);
      target = 2;
      option_button++;
    } else if (option_button == 3) {
      PS4.setLed(50, 205, 50);
      PS4.sendToController();
      target = 0;
      option_button = 1;
    }
  }
  if (event.button_down.ps) {
    Wire.beginTransmission(I2C_DEV_ADDR_shooter);
    Wire.print(9);
    Wire.endTransmission();
  //  Serial.println(9);
  }
  if(event.button_down.triangle)
  {
      Wire.beginTransmission(I2C_DEV_ADDR_shooter);
    Wire.print("b");
    Wire.endTransmission();
    //Serial.println("b");
    
  }
}

double Kp = 0.12, Ki = 4, Kd = 0;
// double Kp = 0.01, Ki = 8.5, Kd = 0.01;
PID myPIDR1(&InputR1, &OutputR1, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDL1(&InputL1, &OutputL1, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDR2(&InputR2, &OutputR2, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDL2(&InputL2, &OutputL2, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ps4SetEventCallback(controller_event_cb);
  // put your setup code here, to run once:
  xTaskCreatePinnedToCore(getrpm, "Task1", 10000, NULL, 1, NULL, 1);

  delay(500);

  xTaskCreatePinnedToCore(loco, "Task1", 10000, NULL, 0, NULL, 0);
  pinMode(anR1, INPUT);
  pinMode(digR1, OUTPUT);
  pinMode(anR2, INPUT);
  pinMode(digR2, OUTPUT);
  pinMode(anL1, INPUT);
  pinMode(digL1, OUTPUT);
  pinMode(anL2, INPUT);
  pinMode(digL2, OUTPUT);

  // Sets the echoPin as an Input
  pinMode(ENCODER_R1_A, INPUT_PULLUP);
  pinMode(ENCODER_R1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R1_A), getEncoderCounter_R1, CHANGE);
  pinMode(ENCODER_R2_A, INPUT_PULLUP);
  pinMode(ENCODER_R2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R2_A), getEncoderCounter_R2, CHANGE);
  pinMode(ENCODER_L1_A, INPUT_PULLUP);
  pinMode(ENCODER_L1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L1_A), getEncoderCounter_L1, CHANGE);
  pinMode(ENCODER_L2_A, INPUT_PULLUP);
  pinMode(ENCODER_L2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L2_A), getEncoderCounter_L2, CHANGE);
  PS4.attachOnConnect(onConnect);
  PS4.begin("b8:d6:1a:67:70:22");  // 7c:9e:bd:47:94:06 joel // b8:d6:1a:67:70:20 nathan
  Setpoint = 150;
  //turn the PID on
  myPIDR2.SetMode(AUTOMATIC);
  myPIDR2.SetOutputLimits(0, 200);
  myPIDR1.SetMode(AUTOMATIC);
  myPIDR1.SetOutputLimits(0, 200);
  myPIDL1.SetMode(AUTOMATIC);
  myPIDL1.SetOutputLimits(0, 200);
  myPIDL2.SetMode(AUTOMATIC);
  myPIDL2.SetOutputLimits(0, 200);

  Wire.begin();
  Serial.println("CLEARSHEET"); // clears sheet starting at row 1  
  // define 5 columns named "Date", "Time", "Timer", "Counter" and "millis"
    Serial.println("LABEL,PWm_l1,Pwm_l2,Pwm_r1,Pwm_r2");
  delay(1000);
}

void loop() {


}
void forward(int rpm) {
  //getRPM();
  InputR1 = abs(rpm_R1);
  InputR2 = abs(rpm_R2);
  InputL1 = abs(rpm_L1);
  InputL2 = abs(rpm_L2);
  computePID();
  Serial.println( (String) "DATA," + OutputL1 + "," +OutputL2 + ","+ OutputR1 + ","+ OutputR2 + ",AUTOSCROLL_20" );
  is_diagonal = 0;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, HIGH);
  digitalWrite(digL2, HIGH);
  digitalWrite(digR1, LOW);
  digitalWrite(digR2, LOW);
  currentConfig[0] = 1;
  currentConfig[1] = 1;
  currentConfig[2] = 0;
  currentConfig[3] = 0;
}

void reverse(int rpm) {
  //getRPM();
  InputR1 = abs(rpm_R1);
  InputR2 = abs(rpm_R2);
  InputL1 = abs(rpm_L1);
  InputL2 = abs(rpm_L2);
  computePID();

  is_diagonal = 0;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, LOW);
  digitalWrite(digL2, LOW);
  digitalWrite(digR1, HIGH);
  digitalWrite(digR2, HIGH);
  currentConfig[0] = 0;
  currentConfig[1] = 0;
  currentConfig[2] = 1;
  currentConfig[3] = 1;
}

void antiClockWise(int rpm) {
  //getRPM();
  is_diagonal = 0;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
 // Serial.println("AntiClockWise: ");
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, LOW);
  digitalWrite(digL2, LOW);
  digitalWrite(digR1, LOW);
  digitalWrite(digR2, LOW);
  currentConfig[0] = 0;
  currentConfig[1] = 0;
  currentConfig[2] = 0;
  currentConfig[3] = 0;
}

void clockWise(int rpm) {
  //getRPM();
  is_diagonal = 0;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  // Serial.println("ClockWise: ");
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, HIGH);
  digitalWrite(digL2, HIGH);
  digitalWrite(digR1, HIGH);
  digitalWrite(digR2, HIGH);
  currentConfig[0] = 1;
  currentConfig[1] = 1;
  currentConfig[2] = 1;
  currentConfig[3] = 1;
}

void leftDiagonal(int rpm) {
  //getRPM();
  is_diagonal = 1;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  // Serial.println("LeftDiagonal: ");
  analogWrite(anL1, 0);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, 0);
  digitalWrite(digL1, HIGH);
  digitalWrite(digL2, HIGH);
  digitalWrite(digR1, LOW);
  digitalWrite(digR2, LOW);
  currentConfig[0] = 1;
  currentConfig[1] = 1;
  currentConfig[2] = 0;
  currentConfig[3] = 0;
}

void rightDiagonal(int rpm) {
  //getRPM();
  is_diagonal = 2;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  // Serial.println("RightDiagonal: ");
  analogWrite(anL1, pwm1);
  analogWrite(anL2, 0);
  analogWrite(anR1, 0);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, HIGH);
  digitalWrite(digL2, HIGH);
  digitalWrite(digR1, LOW);
  digitalWrite(digR2, LOW);
  currentConfig[0] = 1;
  currentConfig[1] = 1;
  currentConfig[2] = 0;
  currentConfig[3] = 0;
}
void strafe(int dir, int rpm) {
  //getRPM();
  is_diagonal = 0;
  //Right 0
  //Left 1
  int gau = (dir == 0) ? 0 : 1;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  // Serial.print("Strafe ");
  // Serial.println((dir == 0) ? "Right" : "Left");

  // Serial.println(rpm_R2);
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, !gau);
  digitalWrite(digL2, (gau));
  digitalWrite(digR1, !(gau));
  digitalWrite(digR2, (gau));
  currentConfig[0] = !gau;
  currentConfig[1] = gau;
  currentConfig[2] = !gau;
  currentConfig[3] = gau;
}

void Stop() {
  //getRPM();

  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  digitalWrite(digL1, currentConfig[0]);
  digitalWrite(digL2, currentConfig[1]);
  digitalWrite(digR1, currentConfig[2]);
  digitalWrite(digR2, currentConfig[3]);

  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
}
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
  InputR1 = abs(rpm_R1);
  InputR2 = abs(rpm_R2);
  InputL1 = abs(rpm_L1);
  InputL2 = abs(rpm_L2);

  if (ryAxis < -sensitivity && abs(rxAxis) < 50) {
    counter = 0;
    computePID();
    rightDiagonal(mryAxis);
  }

  else if (ryAxis > sensitivity && abs(rxAxis) < 50) {
    counter = 0;
    computePID();
    leftDiagonal(mryAxis);
  }

  else if (rxAxis > sensitivity && abs(ryAxis) < 50) {
    counter = 0;
    computePID();
    clockWise(mrxAxis);
  }

  else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {
    counter = 0;
    computePID();
    antiClockWise(mrxAxis);
  }

  else if (lyAxis < -sensitivity && abs(lxAxis) < 50) {
    counter = 0;
    computePID();
    reverse(mlyAxis);
  }

  else if (lyAxis > sensitivity && abs(lxAxis) < 50) {
    counter = 0;
    computePID();
    forward(mlyAxis);
  }

  else if (lxAxis > sensitivity && abs(lyAxis) < 50) {
    counter = 0;
    computePID();
    strafe(0, mlxAxis);
  }

  else if (lxAxis < -sensitivity && abs(lyAxis) < 50) {
    counter = 0;
    computePID();
    strafe(1, mlxAxis);
  } else {
    if (Setpoint < 160) {

      if (is_diagonal == 0) {
        input = Setpoint + counter;
        InputR1 = input;
        InputR2 = input;
        InputL1 = input;
        InputL2 = input;
        is_diagonal = 0;
      } else if (is_diagonal == 1) {
        input = Setpoint + counter;
        InputR1 = input;
        InputL2 = input;
        InputR2 = 500;
        InputL1 = 500;
        is_diagonal = 1;
      } else if (is_diagonal == 2) {
        input = Setpoint + counter;
        InputR2 = input;
        InputL1 = input;
        InputR1 = 500;
        InputL2 = 500;
        is_diagonal = 2;
      }
      counter = counter + 70;
    } else if (Setpoint > 161 && Setpoint < 250) {

      if (is_diagonal == 0) {
        input = Setpoint + counter;
        InputR1 = input;
        InputR2 = input;
        InputL1 = input;
        InputL2 = input;
        is_diagonal = 0;
      } else if (is_diagonal == 1) {
        input = Setpoint + counter;
        InputR1 = input;
        InputL2 = input;
        InputR2 = 500;
        InputL1 = 500;
        is_diagonal = 1;
      } else if (is_diagonal == 2) {
        input = Setpoint + counter;
        InputR2 = input;
        InputL1 = input;
        InputR1 = 500;
        InputL2 = 500;
        is_diagonal = 2;
      }
      counter = counter + 150;
    }

    computePID();
    Stop();
  }
}

void computePID() {
  myPIDR1.Compute();
  myPIDL1.Compute();
  myPIDL2.Compute();
  myPIDR2.Compute();
}

void getDistanceAndLogic() {

  if (isShareON == true) {
    if (target == 1) {
      //ledConditions(getUltraSonicDistance());
    } else if (target == 2) {
      //  ledConditions(getUltraSonicDistance());
    }
  }
}

#include <PID_v1.h>
#include <PS4Controller.h>

int ENCODER_R2_A = 15;  // Pin for Encoder A
int ENCODER_R2_B = 18;
int ENCODER_R1_A = 39;  // Pin for Encoder A
int ENCODER_R1_B = 36;
int ENCODER_L1_A = 35;  // Pin for Encoder A
int ENCODER_L1_B = 34;
int ENCODER_L2_A = 14;  // Pin for Encoder A
int ENCODER_L2_B = 13;

int anL2 = 16;
int digL2 = 17;
int anL1 = 5;
int digL1 = 27;
int anR1 = 26;
int digR1 = 25;
int anR2 = 33;  //13
int digR2 = 32;
// int ENCODER_R2_A = 15;  // Pin for Encoder A
// int ENCODER_R2_B = 5;
// int ENCODER_R1_A = 37;  // Pin for Encoder A
// int ENCODER_R1_B = 36;
// int ENCODER_L1_A = 35;  // Pin for Encoder A
// int ENCODER_L1_B = 34;
// int ENCODER_L2_A = 14;  // Pin for Encoder A
// int ENCODER_L2_B = 12;

// int anR2 = 16;
// int digR2 = 17;
// int anR1 = 13;
// int digR1 = 27;
// int anL1 = 26;  //13
// int digL1 = 25;
// int anL2 = 33;  //13
// int digL2=32;
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
int currentConfig[4] = { 0, 0, 0, 0 };
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
    } else {
      Stop();
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
    // delay(50);
  }
}


void controller_event_cb(ps4_t ps4, ps4_event_t event) {
  if (event.button_down.square) {
    if (!isLocked) {
      Serial.println("-- Controller Locked --");
      isLocked = 1;
    } else {
      Serial.println("-- Controller Unlocked --");
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
  PS4.begin("d4:d4:da:5c:2f:86");  // 7c:9e:bd:47:94:06 joel // b8:d6:1a:67:70:20 nathan
  Setpoint = 150;
  //turn the PID on
  myPIDR2.SetMode(AUTOMATIC);
  myPIDR2.SetOutputLimits(0, 255);
  myPIDR1.SetMode(AUTOMATIC);
  myPIDR1.SetOutputLimits(0, 255);
  myPIDL1.SetMode(AUTOMATIC);
  myPIDL1.SetOutputLimits(0, 255);
  myPIDL2.SetMode(AUTOMATIC);
  myPIDL2.SetOutputLimits(0, 255);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
}
void forward(int rpm) {
  //getRPM();
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
  digitalWrite(digR1, HIGH);
  digitalWrite(digR2, HIGH);
  currentConfig[0] = 1;
  currentConfig[1] = 1;
  currentConfig[2] = 1;
  currentConfig[3] = 1;
}

void reverse(int rpm) {
  //getRPM();
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
  digitalWrite(digR1, LOW);
  digitalWrite(digR2, LOW);
  currentConfig[0] = 0;
  currentConfig[1] = 0;
  currentConfig[2] = 0;
  currentConfig[3] = 0;
}

void antiClockWise(int rpm) {
  //getRPM();
  is_diagonal = 0;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  Serial.println("AntiClockWise: ");
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

void clockWise(int rpm) {
  //getRPM();
  is_diagonal = 0;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  Serial.println("ClockWise: ");
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

void leftDiagonal(int rpm) {
  //getRPM();
  is_diagonal = 1;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  Serial.println("LeftDiagonal: ");
  analogWrite(anL1, 0);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, 0);
  digitalWrite(digL1, HIGH);
  digitalWrite(digL2, HIGH);
  digitalWrite(digR1, HIGH);
  digitalWrite(digR2, HIGH);
  currentConfig[0] = 1;
  currentConfig[1] = 1;
  currentConfig[2] = 1;
  currentConfig[3] = 1;
}

void rightDiagonal(int rpm) {
  //getRPM();
  is_diagonal = 2;
  int pwm1 = OutputL1;
  int pwm2 = OutputL2;
  int pwm3 = OutputR1;
  int pwm4 = OutputR2;
  Serial.println("RightDiagonal: ");
  analogWrite(anL1, pwm1);
  analogWrite(anL2, 0);
  analogWrite(anR1, 0);
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
  Serial.print("Strafe ");
  Serial.println((dir == 0) ? "Right" : "Left");

  Serial.println(rpm_R2);
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, !gau);
  digitalWrite(digL2, !(!gau));
  digitalWrite(digR1, (gau));
  digitalWrite(digR2, !(gau));
  currentConfig[0] = !gau;
  currentConfig[1] = gau;
  currentConfig[2] = gau;
  currentConfig[3] = !gau;
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

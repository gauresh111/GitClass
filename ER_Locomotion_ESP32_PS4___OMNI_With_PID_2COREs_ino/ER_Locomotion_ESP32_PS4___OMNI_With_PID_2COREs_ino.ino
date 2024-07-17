#include <PID_v1.h>
#include <PS4Controller.h>
#include <Wire.h>
int ENCODER_R1_A = 25;  // Pin for Encoder A
int ENCODER_R1_B = 26;
int ENCODER_R2_A = 35;  // Pin for Encoder A
int ENCODER_R2_B = 34;
int ENCODER_L1_A = 33;  // Pin for Encoder A
int ENCODER_L1_B = 32;
int ENCODER_L2_A = 39;  // Pin for Encoder A
int ENCODER_L2_B = 36;

int anL1 = 5;    //5
int digL1 = 19;  //27
int anR1 = 17;
int digR1 = 18;
int anL2 = 13;
int digL2 = 27;
int anR2 = 14;  //13
int digR2 = 23;
int shotter_digital = 16;
int shotter_analog = 2;
int acuator_analog = 15;
int acuator_digital = 4;
volatile int encoderValueSet_R2 = 0;
float rpm_R2 = 0;
volatile int encoderValueSet_R1 = 0;
float rpm_R1 = 0;
volatile int encoderValueSet_L1 = 0;
float rpm_L1 = 0;
volatile int encoderValueSet_L2 = 0;
float rpm_L2 = 0;
int RPM = 100;
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
#define I2C_DEV_ADDR 0x55
int right_checked = 0;
int left_checked = 0;
int triangle_checked = 0;
int cross_checked = 0;
bool place_motor = false;
bool place_stop = false;
bool lift_motor = false;
bool lift_stop = false;

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
      // Serial.print("Target:");
    // Serial.print(Setpoint);
    // Serial.print(",");
    // Serial.print("R1_rpm:");
    // Serial.print(abs(rpm_R1));
    // Serial.print(",");
    // Serial.print("R2_rpm:");
    // Serial.print(abs(rpm_R2));
    // Serial.print(",");
    // Serial.print("L1_rpm:");
    // Serial.print(abs(rpm_L1));
    // Serial.print(",");
    // Serial.print("L2_rpm:");
    // Serial.println(abs(rpm_L2));

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

    while (endTime - startTime < 125) {
      endTime = millis();
      // getEncoderCounter();
    }
    int lineCount = 191;  //650 for 750 rpm motor else 267
    rpm_R2 = (encoderValueSet_R2 * 8 * 60) / lineCount;
    rpm_R1 = (encoderValueSet_R1 * 8 * 60) / lineCount;
    rpm_L1 = (encoderValueSet_L1 * 8 * 60) / lineCount;
    rpm_L2 = (encoderValueSet_L2 * 8 * 60) / lineCount;

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
    if (Setpoint <= 180) {
      Setpoint = Setpoint + 20;
      Serial.println((String) "-- Setpoint Incremented: " + Setpoint + " --");
    }
  }
  if (event.button_down.down) {
    if (Setpoint >= 60) {
      Setpoint = Setpoint - 20;
      Serial.println((String) "-- Setpoint Decremented: " + Setpoint + " --");
    }
  }
  if (event.button_down.circle) {
    Wire.beginTransmission(I2C_DEV_ADDR);
    Wire.print(5);
    Serial.println(5);
    Wire.endTransmission();
  }
}

double Kp = 0.12, Ki = 2.5, Kd = 0;

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
  PS4.begin("7c:66:ef:cd:43:70");  // 7c:9e:bd:47:94:06 joel // b8:d6:1a:67:70:20 nathan
  Setpoint = 100;
  //turn the PID on
  myPIDR2.SetMode(AUTOMATIC);
  myPIDR2.SetOutputLimits(0, 255);
  myPIDR1.SetMode(AUTOMATIC);
  myPIDR1.SetOutputLimits(0, 255);
  myPIDL1.SetMode(AUTOMATIC);
  myPIDL1.SetOutputLimits(0, 255);
  myPIDL2.SetMode(AUTOMATIC);
  myPIDL2.SetOutputLimits(0, 255);
  pinMode(shotter_digital, OUTPUT);
  pinMode(shotter_analog, INPUT);
  pinMode(acuator_analog, INPUT);
  pinMode(acuator_digital, OUTPUT);
  move_acuator(0, 0);
  move_shooter(0, 0);
  Wire.begin();

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(" lauda");

  if(PS4.L1())
  {
    move_shooter(230,1);
  }
  else if (PS4.R1())
  {
    move_shooter(230,0);
  }
  else {
    move_shooter(0,0);
  }
  if(PS4.L2())
  {
    move_acuator(255,1);
  }
  else if (PS4.R2())
  {
    move_acuator(255,0);
  }
  else 
  {
    move_acuator(0,0);
  }
  if (PS4.Right()) {
    if (place_motor == false) {
      Wire.beginTransmission(I2C_DEV_ADDR);
      Wire.print(1);
      Serial.println(1);
      Wire.endTransmission();
      place_stop = false;
      place_motor = true;
    }
  } else if (PS4.Left()) {
    if (place_motor == false) {
      Wire.beginTransmission(I2C_DEV_ADDR);
      Wire.print(2);
      Serial.println(2);
      Wire.endTransmission();
      place_stop = false;
      place_motor = true;
    }
  } else {
    //move_acuator(0, 1);
    if (place_stop == false) {
      Wire.beginTransmission(I2C_DEV_ADDR);
      Wire.print(3);
      Serial.println(3);
      Wire.endTransmission();
      place_stop = true;
      place_motor = false;
    }
  }
  if (PS4.Triangle()) {
    if (lift_motor == false) {
      Wire.beginTransmission(I2C_DEV_ADDR);
      Wire.print(4);
      Serial.println(4);
      Wire.endTransmission();
      lift_motor = true;
      lift_stop = false;
    }
  } else if (PS4.Cross()) {
    if (lift_motor == false) {
      Wire.beginTransmission(I2C_DEV_ADDR);
      Wire.print(6);
      Serial.println(6);
      Wire.endTransmission();
      lift_motor = true;
      lift_stop = false;
    }
  } else {
    if (lift_stop == false) {
      Wire.beginTransmission(I2C_DEV_ADDR);
      Wire.print(7);
      Serial.println(7);
      Wire.endTransmission();
      lift_stop = true;
      lift_motor = false;
    }
  }
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
  digitalWrite(digL1, 0);
  digitalWrite(digL2, 1);
  digitalWrite(digR1, 0);
  digitalWrite(digR2, 0);
  currentConfig[0] = 0;
  currentConfig[1] = 1;
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
  Serial.println("ClockWise: ");
  analogWrite(anL1, pwm1);
  analogWrite(anL2, pwm2);
  analogWrite(anR1, pwm3);
  analogWrite(anR2, pwm4);
  digitalWrite(digL1, 1);
  digitalWrite(digL2, 0);
  digitalWrite(digR1, 1);
  digitalWrite(digR2, 1);
  currentConfig[0] = 1;
  currentConfig[1] = 0;
  currentConfig[2] = 1;
  currentConfig[3] = 1;
}

void leftDiagonal(int rpm, int dir) {
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
  digitalWrite(digL1, dir);
  digitalWrite(digL2, !dir);
  digitalWrite(digR1, !dir);
  digitalWrite(digR2, !dir);
  currentConfig[0] = 0;
  currentConfig[1] = !dir;
  currentConfig[2] = !dir;
  currentConfig[3] = 0;
}

void rightDiagonal(int rpm, int dir) {
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
  digitalWrite(digL1, dir);
  digitalWrite(digL2, dir);
  digitalWrite(digR1, !dir);
  digitalWrite(digR2, dir);
  currentConfig[0] = 0;
  currentConfig[1] = 0;
  currentConfig[2] = !dir;
  currentConfig[3] = dir;
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
void move_shooter(int speed, int direction) {
  analogWrite(shotter_analog, speed);
  digitalWrite(shotter_digital, direction);
}
void move_acuator(int speed, int direction) {
  analogWrite(acuator_analog, speed);
  digitalWrite(acuator_digital, direction);
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

  if (lxAxis > sensitivity && abs(lyAxis) < 50) {
    counter = 0;
    computePID();
    clockWise(Setpoint);
  }

  else if (lxAxis < -sensitivity && abs(lyAxis) < 50) {
    counter = 0;
    computePID();
    antiClockWise(Setpoint);
  }

  else if (ryAxis < -sensitivity && abs(rxAxis) < 50) {
    counter = 0;
    computePID();
    leftDiagonal(Setpoint, 0);
    Serial.print("backward");
  }

  else if (ryAxis > sensitivity && abs(rxAxis) < 50) {
    counter = 0;
    computePID();
    leftDiagonal(Setpoint, 1);
    Serial.print("forward");
  }

  else if (rxAxis > sensitivity && abs(ryAxis) < 50) {
    counter = 0;
    computePID();
    rightDiagonal(Setpoint, 1);
    Serial.print("left");
  }

  else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {
    counter = 0;
    computePID();
    rightDiagonal(Setpoint, 0);
    Serial.print("right");
  } else {
    InputR2 = 500;
    InputL1 = 500;
    InputR1 = 500;
    InputL2 = 500;
    computePID();
    Stop();
    //counter = counter + 30;
  }
}

void computePID() {
  myPIDR1.Compute();
  myPIDL1.Compute();
  myPIDL2.Compute();
  myPIDR2.Compute();
}

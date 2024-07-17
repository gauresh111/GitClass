/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
int ENCODER_R2_A = 15;  // Pin for Encoder A
int ENCODER_R2_B = 18;
int ENCODER_R1_A = 39;  // Pin for Encoder A
int ENCODER_R1_B = 36;
int ENCODER_L2_A = 35;  // Pin for Encoder A
int ENCODER_L2_B = 34;
int ENCODER_L1_A = 14;  // Pin for Encoder A
int ENCODER_L1_B = 13;

int enB_L2 = 16;
int in_L2 = 17;
int enB_L1 = 5;
int in_L1 = 27;
int enB_R1 = 26;
int in_R1 = 25;
int enB_R2 = 33;  //13
int in_R2 = 32;

volatile int encoderValueSet_R2 = 0;
float rpm_R2 = 0;
volatile int encoderValueSet_R1 = 0;
float rpm_R1 = 0;
volatile int encoderValueSet_L1 = 0;
float rpm_L1 = 0;
volatile int encoderValueSet_L2 = 0;
float rpm_L2 = 0;
int counter = 0;
// PWM = Pin 3, DIR = Pin 4.

void getEncoderCounter_R1() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_R1_A);
  int B = digitalRead(ENCODER_R1_B);

  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_R1--;

    //counter++;
  } else {
    encoderValueSet_R1++;
  }
}
void getEncoderCounter_R2() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_R2_A);
  int B = digitalRead(ENCODER_R2_B);

  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_R2--;
    //  counter++;
  } else {
    encoderValueSet_R2++;
  }
}
void getEncoderCounter_L1() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_L1_A);
  int B = digitalRead(ENCODER_L1_B);

  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_L1--;
    //counter++;
  } else {
    encoderValueSet_L1++;
  }
}
void getEncoderCounter_L2() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_L2_A);
  int B = digitalRead(ENCODER_L2_B);

  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoderValueSet_L2--;
    //counter++;
  } else {
    encoderValueSet_L2++;
  }
}
void getRPM() {
  int startTime = millis();
    int endTime = startTime;

    while (endTime - startTime < 100) {
      endTime = millis();
      // getEncoderCounter();
    }
    int lineCount = 267;  //650 for 750 rpm motor else 267
    rpm_R2 = (encoderValueSet_R2 * 10 * 60) / lineCount;
    rpm_R1 = (encoderValueSet_R1 * 10 * 60) / lineCount;
    rpm_L1 = (encoderValueSet_L1 * 10 * 60) / lineCount;
    rpm_L2 = (encoderValueSet_L2 * 10 * 60) / lineCount;

    encoderValueSet_R2 = 0;
    encoderValueSet_R1 = 0;
    encoderValueSet_L1 = 0;
    encoderValueSet_L2 = 0;
}

//Define Variables we'll be connecting to
double Setpoint, InputR2, OutputR2;
double InputR1, OutputR1;
double InputL1, OutputL1;
double InputL2, OutputL2;
//Specify the links and initial tuning parameters
//double Kp = 0.12, Ki =2.5 , Kd = 0; // perfect values
//double Kp = 0.58, Ki = 5, Kd = 0.001;  //4.5
double Kp = 0, Ki = 0, Kd = 0;
//double Kp = 0.16, Ki = 3.8, Kd = 0;
PID myPIDR1(&InputR1, &OutputR1, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDR2(&InputR2, &OutputR2, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDL1(&InputL1, &OutputL1, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDL2(&InputL2, &OutputL2, &Setpoint, Kp, Ki, Kd, DIRECT);

int initialTime = millis();
int finalTime = initialTime;
int F = 0;

void setup() {
  //initialize the variables we're linked to
  // pinMode(ENB, OUTPUT);
  // pinMode(IN2, OUTPUT);
  pinMode(enB_R1, INPUT);
  pinMode(in_R1, OUTPUT);
  pinMode(enB_R2, INPUT);
  pinMode(in_R2, OUTPUT);

  pinMode(enB_L1, INPUT);
  pinMode(in_L1, OUTPUT);
  pinMode(in_L1, LOW);
  pinMode(enB_L2, INPUT);
  pinMode(in_L2, OUTPUT);
  pinMode(in_L2, LOW);
  delay(1000);
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
  Serial.begin(9600);

  Setpoint = 150;
  //turn the PID on
  myPIDR1.SetMode(AUTOMATIC);
  myPIDR1.SetOutputLimits(0, 255);
  // myPIDl1.AutoTune(RELAY, 10);
  myPIDR2.SetMode(AUTOMATIC);
  myPIDR2.SetOutputLimits(0, 255);
  myPIDL1.SetMode(AUTOMATIC);
  myPIDL1.SetOutputLimits(0, 200);
  myPIDL2.SetMode(AUTOMATIC);
  myPIDL2.SetOutputLimits(0, 200);
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,Time,Current_RPM_R1,Target");
  // Serial.println("Ready");

  int counter = 0;
  // Serial.println("LABEL,Date,Time,Timer,Counter,millis");
}

void loop() {

  int startTime = millis();
  int endTime = startTime;
  F = 0;

  while (endTime - startTime < 1000) {
    endTime = millis();
    getRPM();
    InputR1 = abs(rpm_R1);
    InputR2 = abs(rpm_R2);
    InputL1 = abs(rpm_L1);
    InputL2 = abs(rpm_L2);

    // Compute output value from PID controller
    myPIDR1.Compute();
    myPIDR2.Compute();
    myPIDL1.Compute();
    myPIDL2.Compute();

    analogWrite(enB_R2, 100);
    digitalWrite(in_R2, HIGH);
    analogWrite(enB_R1, OutputR1);
    digitalWrite(in_R1, HIGH);
    analogWrite(enB_L1, 100);
    digitalWrite(in_L1, HIGH);
    analogWrite(enB_L2, OutputL2);
    digitalWrite(in_L2, HIGH);
    //Serial.println(Output);
    // Turn on motor A & B
    //Serial.println((String) "DATA," + millis() + ","+ abs(rpm_R1) + ","+ Setpoint + ",AUTOSCROLL_20");
    // Serial.print("TARGET:");
    // Serial.print(Setpoint);
    // Serial.print(",");
    // Serial.print("PWM:");
    // Serial.print(OutputR1);
    // Serial.print(",");
    //kp:0.12,ki:6.50,kd:0.00
    // Serial.print("R1_rpm:");
    // Serial.print(abs(rpm_R1));
    // Serial.print(",");
    // Serial.print("R2_rpm:");
    // Serial.print(abs(rpm_R2));
    // Serial.print(",");
    // Serial.print("    kp:");
    // Serial.print(myPIDR1.GetKp());
    // Serial.print(",");
    // Serial.print("ki:");
    // Serial.print(myPIDR1.GetKi());
    // Serial.print(",");
    // Serial.print("kd:");
    // Serial.println(myPIDR1.GetKd());
    // Serial.print(",");
    // Serial.print("TIME:");
    // Serial.println(F);

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
    if (counter >= 50) {
      Setpoint = random(50, 180);
      counter = 0;
    }
    if (Serial.available() > 0) {
      // read the incoming byte:


      int temp = Serial.parseInt();
      //   temp = temp.trim();

      bool happened = false;
      if (temp != 0) {
        Serial.println(temp);
        switch (temp) {
          case 1:

            while (happened == false) {

              double temp = Serial.parseFloat();
              if (temp != 0) {
                Kp = temp;
                myPIDR1.SetTunings(Kp, Ki, Kd);
                myPIDR2.SetTunings(Kp, Ki, Kd);
                myPIDL1.SetTunings(Kp, Ki, Kd);
                myPIDL2.SetTunings(Kp, Ki, Kd);
                InputR1 = abs(rpm_R1);
                InputR2 = abs(rpm_R2);
                InputL1 = abs(rpm_L1);
                InputL2 = abs(rpm_L2);
                myPIDR1.Compute();
                myPIDR2.Compute();
                myPIDL1.Compute();
                myPIDL2.Compute();
                


                analogWrite(enB_R2, OutputR1);
                digitalWrite(in_R2, HIGH);
                analogWrite(enB_R1, OutputR2);
                digitalWrite(in_R1, HIGH);
                analogWrite(enB_L1, OutputL1);
                digitalWrite(in_L1, HIGH);
                analogWrite(enB_L2, OutputL2);
                digitalWrite(in_L2, HIGH);
                Serial.println((String) "Kp: " + myPIDR1.GetKp());
                delay(1000);
                happened = true;
              }
            }
            break;
          case 2:

            while (happened == false) {


              double temp = Serial.parseFloat();
              if (temp != 0) {
                Ki = temp;

                myPIDR1.SetTunings(Kp, Ki, Kd);
                myPIDR2.SetTunings(Kp, Ki, Kd);
                myPIDL1.SetTunings(Kp, Ki, Kd);
                myPIDL2.SetTunings(Kp, Ki, Kd);
                InputR1 = abs(rpm_R1);
                InputR2 = abs(rpm_R2);
                InputL1 = abs(rpm_L1);
                InputL2 = abs(rpm_L2);
                myPIDR1.Compute();
                myPIDR2.Compute();
                myPIDL1.Compute();
                myPIDL2.Compute();
                
                analogWrite(enB_R2, OutputR1);
                digitalWrite(in_R2, HIGH);
                analogWrite(enB_R1, OutputR2);
                digitalWrite(in_R1, HIGH);
                analogWrite(enB_L1, OutputL1);
                digitalWrite(in_L1, HIGH);
                analogWrite(enB_L2, OutputL2);
                digitalWrite(in_L2, HIGH);
                Serial.println((String) "Ki: " + myPIDR1.GetKi());
                delay(1000);
                //0.11,3.2,0.00
                happened = true;
              }
            }
            break;

          case 3:

            while (happened == false) {

              double temp = Serial.parseFloat();
              if (temp != 0) {
                Kd = temp;
                myPIDR1.SetTunings(Kp, Ki, Kd);
                myPIDR2.SetTunings(Kp, Ki, Kd);
                myPIDL1.SetTunings(Kp, Ki, Kd);
                myPIDL2.SetTunings(Kp, Ki, Kd);
                InputR1 = abs(rpm_R1);
                InputR2 = abs(rpm_R2);
                InputL1 = abs(rpm_L1);
                InputL2 = abs(rpm_L2);
                myPIDR1.Compute();
                myPIDR2.Compute();
                myPIDL1.Compute();
                myPIDL2.Compute();


                analogWrite(enB_R2, OutputR1);
                digitalWrite(in_R2, HIGH);
                analogWrite(enB_R1, OutputR2);
                digitalWrite(in_R1, HIGH);
                analogWrite(enB_L1, OutputL1);
                digitalWrite(in_L1, HIGH);
                analogWrite(enB_L2, OutputL2);
                digitalWrite(in_L2, HIGH);
                Serial.println((String) "Kd: " + myPIDR1.GetKd());

                delay(1000);
                happened = true;
              }
            }
            break;

          case 4:

            while (happened == false) {

              int temp = Serial.parseInt();
              getRPM();
              InputR1 = abs(rpm_R1);
              rpm_R1 = 0;
              // Compute output value from PID controller
              myPIDR1.Compute();
              myPIDR2.Compute();

              if (temp != 0) {
                Setpoint = temp;
                Serial.println((String) "Setpoint: " + Setpoint);
                happened = true;
              }
            }
            break;
        }

        // say what you got:
      }
    }
    counter = counter + 1;

    // getEncoderCounter();
  }
  F = 500;
  Serial.print(",");
  Serial.print("TIME:");
  Serial.println(F);
}

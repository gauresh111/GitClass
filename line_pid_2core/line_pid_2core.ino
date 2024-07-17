#include <PS4Controller.h>

int arrayOfPins[] = { 26, 36, 25, 39, 33, 34, 32, 35 };
// int mazeArray[]
int J_Pulse = 23;
int lsaInputs[8];
int anR1 = 5;
int digR1 = 18;
int anL1 = 16;  //13
int digL1 = 17;
int sensitivity = 60;
int PWM = 100;
int oneInch = 100;
int left_pwm = PWM;
int right_pwm = PWM;
int positionVal;
bool lsa_work = true;
double sensed_output, control_signal;
float setPoint = 3.5;
double Kp = 8;  //proportional gain
double Ki = 0;    //integral gain
double Kd = 0;    //derivative gain
int T;            //sample time in milliseconds (ms)
unsigned long last_time;
double total_error, last_error;
int max_control;
int min_control;
float lastError = 0;
float error;
//core 2

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;


void getlsa(void* pvParameters) {


  for (;;) {
    readLSAInputs();
    error = get_postival();
  }
}

void controller_event_cb(ps4_t ps4, ps4_event_t event) {
  if (event.button_down.cross) {

    if (lsa_work) {
      lsa_work = false;
      Serial.println("lsa-OFF");
    } else {
      Serial.println("lsa-ON");
      lsa_work = true;
    }
  }
  if (event.button_down.up && PS4.Circle()) {
    Serial.println("-- Setpoint Incremented--");
    if (Kp >= 0.1) {
      Kp = Kp - 0.1;
    }
  } else if (event.button_down.up) {
    Kp = Kp + 0.1;
  }

  if (event.button_down.left && PS4.Circle()) {
    Serial.println("-- Setpoint Incremented--");
    if (Ki >= 0.1) {
      Ki = Ki - 0.1;
    }
  } else if (event.button_down.left) {
    Ki = Ki + 0.1;
  }
  if (event.button_down.right && PS4.Circle()) {
    Serial.println("-- Setpoint Incremented--");
    if (Kd >= 0.1) {
      Kd = Kd - 0.1;
    }
  } else if (event.button_down.right) {
    Kd = Kd + 0.1;
  }
  if (event.button_down.triangle) {
    Stop();
    lsa_work = false;
  }
  //   Serial.println(1);
}

void setupPinMode() {
  pinMode(anR1, INPUT);
  pinMode(digR1, OUTPUT);
  pinMode(anL1, INPUT);
  pinMode(digL1, OUTPUT);
  Stop();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  xTaskCreatePinnedToCore(getlsa, "Task1", 10000, NULL, 1, NULL, 1);

  setupPinMode();
  for (int i = 0; i < 8; i++) {
    pinMode(arrayOfPins[i], INPUT);
  }
  pinMode(J_Pulse, INPUT);
  PS4.begin("b8:d6:1a:67:70:22");
  ps4SetEventCallback(controller_event_cb);
  SerialBT.begin("ESP32test");
}
void printLSAValues() {
  Serial.printf("ir0:%d ir1:%d ir2:%d ir3:%d ir4:%d ir5:%d ir6:%d ir7:%d ",
                digitalRead(arrayOfPins[0]),
                digitalRead(arrayOfPins[1]),
                digitalRead(arrayOfPins[2]),
                digitalRead(arrayOfPins[3]),
                digitalRead(arrayOfPins[4]),
                digitalRead(arrayOfPins[5]),
                digitalRead(arrayOfPins[6]),
                digitalRead(arrayOfPins[7]));
Serial.printf("positionVal: %f \n", get_postival());
}

void readLSAInputs() {
  for (int i = 0; i < 8; i++) {
    lsaInputs[i] = digitalRead(arrayOfPins[i]);
  }
}



int detectJunctions() {

  // 2: L - Right
  //-2: L - Left
  // 3: T Junction
  // 4: 4 Way

  //Mid Point :2 - 3 - 4 - 5

  Stop();
  // readLSAInputs();
  int junction = digitalRead(J_Pulse);
  if (junction) {
    int previousLSA[8];
    for (int i = 0; i < 8; i++) {
      previousLSA[i] = lsaInputs[i];
    }
    goOneInch();
    Serial.println("Reading new Values");
    //readLSAInputs();

    Serial.printf("%d   %d\n", previousLSA[0], previousLSA[7]);
    if (lsaInputs[2] == 1 || lsaInputs[3] == 1 || lsaInputs[4] == 1 || lsaInputs[5] == 1) {
      if (previousLSA[0] == previousLSA[7] && previousLSA[1] == previousLSA[6] && previousLSA[0] == 1) {  // all are 1
        return 4;
      } else {
        if (previousLSA[0] == 1 && previousLSA[1] == 1) {
          return -2;
        } else {
          return 2;
        }
      }
    } else {
      return 3;
    }
  }
  return -1;
}
void loop() {
  // put your main code here, to run repeatedly:
  // readLSAInputs();
  // printLSAValues();
  if (lsa_work) {
   // Serial.println("lsa_work");
    isLINE();
  } else {
    //Serial.println("ps4_work");
    Locomote();
  }

  //Forward(255, 255);
  SerialBT.print("Kp :");
  SerialBT.print(Kp);
  SerialBT.print("     Ki :");
  SerialBT.print(Ki);
  SerialBT.print("     Kd :");
  SerialBT.println(Kd);
  delay(10);
}

void Forward(int left, int right) {
   Serial.println("---Forward ---");
  analogWrite(anR1, right);
  analogWrite(anL1, left);
  digitalWrite(digL1, HIGH);
  digitalWrite(digR1, LOW);
}
void Reverse(int left, int right) {
  // Serial.println("---Forward ---");
  analogWrite(anR1, right);
  analogWrite(anL1, left);
  digitalWrite(digL1, LOW);
  digitalWrite(digR1, HIGH);
}
void Stop() {
  // Serial.println("---Forward ---");
  analogWrite(anR1, 0);
  analogWrite(anL1, 0);
  digitalWrite(digL1, HIGH);
  digitalWrite(digR1, HIGH);
}

void Left(int left, int right) {
  // Serial.println("---Forward ---");
  analogWrite(anR1, right);
  analogWrite(anL1, left);
  digitalWrite(digL1, HIGH);
  digitalWrite(digR1, HIGH);
}
void Right(int left, int right) {
  // Serial.println("---Forward ---");
  analogWrite(anR1, right);
  analogWrite(anL1, left);
  digitalWrite(digL1, LOW);
  digitalWrite(digR1, LOW);
}

void goOneInch() {
  Forward(left_pwm, right_pwm);
  Serial.println(" Now Move Forward");
  delay(1000);
  Serial.println("Now Stop");
  Stop();
}

// void straighten(){
//   readLSAInputs();
//   if(lsaInputs[2] && lsaInputs[3])
// }

void PidControl(float positionVal) {

  float error = positionVal - setPoint;                                          // Calculate the deviation from position to the set point
  float motorSpeed = Kp * error + Kd * (error - lastError) + Ki * (Ki + error);  // Applying formula of PID
  lastError = error;                                                           // Store current error as previous error for next iteration use

  // Adjust the motor speed based on calculated value
  // You might need to interchange the + and - sign if your robot move in opposite direction
  float rightMotorSpeed = PWM - motorSpeed;
  float leftMotorSpeed = PWM + motorSpeed;

  // If the speed of motor exceed max speed, set the speed to max speed
  if (rightMotorSpeed > 149) rightMotorSpeed = 150;
  if (leftMotorSpeed > 149) leftMotorSpeed = 150;

  // If the speed of motor is negative, set it to 0
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  left_pwm = leftMotorSpeed;
  right_pwm = rightMotorSpeed;
   Serial.printf("Right Motor:%f , Left Motor: %f   positionVal:%f \n ", rightMotorSpeed, leftMotorSpeed,positionVal);
}
void isLINE() {
  if (lsaInputs[1] == 1 || lsaInputs[2] == 1 || lsaInputs[3] == 1 || lsaInputs[4] == 1 || lsaInputs[5] == 1
      || lsaInputs[6] == 1) {
    Forward(left_pwm, right_pwm);
    if(error != -1){
    PidControl(error);
    }
  //  printLSAValues();
    

  } else {
    Forward(left_pwm, right_pwm);
  }
}

float get_postival() {

  float count_l = 0;
  float count_r = 0;
  float l_times = 0;
  float r_times = 0;
  for (int i = 1, j = 4; i <= 3; i++, j++) {
    if (lsaInputs[i] == 1) {
      count_l += i;
      l_times++;
    }
    if (lsaInputs[j] == 1) {
      count_r += j;
      r_times++;
    }
  }
  if((count_l + count_r)==0 && (l_times + r_times)==0)
  {
    return -1;
  }
  float positionVal = ((float)(count_l + count_r) / (l_times + r_times));
  // make it avg;
  return positionVal;
}
void Locomote() {
  int lxAxis = PS4.LStickX();
  int lyAxis = PS4.LStickY() * (1);
  int rxAxis = PS4.RStickX();
  int ryAxis = PS4.RStickY() * (1);
  if (lyAxis < -sensitivity && abs(lxAxis) < 50) {
    Reverse(100, 100);
  }

  else if (lyAxis > sensitivity && abs(lxAxis) < 50) {
    Forward(100, 100);
  }

  else if (lxAxis > sensitivity && abs(lyAxis) < 50) {
    Left(100, 100);
  }

  else if (lxAxis < -sensitivity && abs(lyAxis) < 50) {
    Right(100, 100);
  } else {

    Stop();
  }
}
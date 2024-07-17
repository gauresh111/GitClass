#include <PS4Controller.h>

int arrayOfPins[] = { 36, 33, 39, 25, 34, 26, 35, 27 };
//motor
int an2 = 18;
int dig2 = 23;
int an1 = 19;
int dig1 = 22;
int PWM = 100;
int left_pwm = PWM;
int right_pwm = PWM;
int sensitivity = 60;
//pid
double Kp = 2;  //proportional gain
double Ki = 0;  //integral gain
double Kd = 0;  //derivative gain
int setPoint = 4095 / 2;
double total_error, last_error;
float lastError = 0;
float error;
//lsa-08
int lsaInputs[8];
bool lsa_work = true;
char address = 0x01;  // UART address as 0x01
int junction = 2;
int analog_value =32;
int postion;

//Bluetooth
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

void PidControl(float positionVal) {

  float error = positionVal - setPoint;                                          // Calculate the deviation from position to the set point
  float motorSpeed = Kp * error + Kd * (error - lastError) + Ki * (Ki + error);  // Applying formula of PID
  lastError = error;                                                             // Store current error as previous error for next iteration use

  // Adjust the motor speed based on calculated value
  // You might need to interchange the + and - sign if your robot move in opposite direction
  float rightMotorSpeed = PWM - motorSpeed;
  float leftMotorSpeed = PWM + motorSpeed;

  rightMotorSpeed = constrain(rightMotorSpeed, 0, 200);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 200);
  left_pwm = leftMotorSpeed;
  right_pwm = rightMotorSpeed;
 // Serial.printf("Right Motor:%f , Left Motor: %f   positionVal:%f \n ", rightMotorSpeed, leftMotorSpeed, positionVal);
}

void sendCommand(char command, char data) {

  char checksum = address + command + data;

  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(checksum);
}

void calibrate() {
  char command, data;
  command = 'C';
  data = 0x00;
  sendCommand(command, data);
  clockwise(200, 200);
  delay(5000);
  stop();
}

void printLSAValues() {
  Serial.printf("ir0:%d ir1:%d ir2:%d ir3:%d ir4:%d ir5:%d ir6:%d ir7:%d \n",
                digitalRead(arrayOfPins[0]),
                digitalRead(arrayOfPins[1]),
                digitalRead(arrayOfPins[2]),
                digitalRead(arrayOfPins[3]),
                digitalRead(arrayOfPins[4]),
                digitalRead(arrayOfPins[5]),
                digitalRead(arrayOfPins[6]),
                digitalRead(arrayOfPins[7]));
  // Serial.print("Analog Value: ");
  // Serial.println(postion);
}

void readLSAInputs() {
  for (int i = 0; i < 8; i++) {
    lsaInputs[i] = digitalRead(arrayOfPins[i]);
  }
  
}

void getlsa(void* pvParameters) {


  for (;;) {
    readLSAInputs();
    
    //delay(1000);
    //printLSAValues();
  }
}
//PS4
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
    stop();
    lsa_work = false;
  }
  if (event.button_down.ps) {
    calibrate();
  }
  if (event.button_down.triangle) {
    stop();
    lsa_work = false;
  }
  //   Serial.println(1);
}

void setup_pinMODE() {
  pinMode(an1, INPUT);
  pinMode(an2, INPUT);
  pinMode(dig1, OUTPUT);
  pinMode(dig2, OUTPUT);
  pinMode(analog_value, INPUT);
  // for (int i = 0; i < 8; i++) {
  //   pinMode(arrayOfPins[i], INPUT);
  // }
  stop();
}
void backward(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, LOW);
  digitalWrite(dig2, LOW);
  Serial.println("### --- backward");
}
void stop() {
  analogWrite(an1, 0);
  analogWrite(an2, 0);
  digitalWrite(dig1, HIGH);
  digitalWrite(dig2, HIGH);
}
void forward(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, HIGH);
  digitalWrite(dig2, HIGH);
 Serial.println("forward");
}
void left(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, LOW);
  digitalWrite(dig2, HIGH);
}
void right(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, HIGH);
  digitalWrite(dig2, LOW);
}
void clockwise(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, HIGH);
  digitalWrite(dig2, LOW);
  Serial.println("Clockwise");
}
void anti_clockwise(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, LOW);
  digitalWrite(dig2, HIGH);
  Serial.println("AntiClockwise");
}
void goOneInch() {
  forward(left_pwm, right_pwm);
  Serial.println(" Now Move Forward");
  delay(1000);
  Serial.println("Now Stop");
  stop();
}

void Locomote() {
  int lxAxis = PS4.LStickX();
  int lyAxis = PS4.LStickY() * (1);
  int rxAxis = PS4.RStickX();
  int ryAxis = PS4.RStickY() * (1);
  if (lyAxis < -sensitivity && abs(lxAxis) < 50) {
    backward(150,150);
  }

  else if (lyAxis > sensitivity && abs(lxAxis) < 50) {
    forward(150, 150);
  }

  else if (lxAxis > sensitivity && abs(lyAxis) < 50) {
    left(150, 150);
  } else if (lxAxis < -sensitivity && abs(lyAxis) < 50) {
    right(150, 150);
  }
  else if (rxAxis > sensitivity && abs(ryAxis) < 50) {
    clockwise(150, 150);
  } else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {

    anti_clockwise(150, 150);
  }

  else {

    stop();
  }
}


void isLINE() {
  if (lsaInputs[1] == 1 || lsaInputs[2] == 1 || lsaInputs[3] == 1 || lsaInputs[4] == 1 || lsaInputs[5] == 1
      || lsaInputs[6] == 1) {
   // forward(left, right);
     forward(left_pwm, right_pwm);
    PidControl(postion);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 16, 17);

  setup_pinMODE();
  xTaskCreatePinnedToCore(getlsa, "Task1", 10000, NULL, 1, NULL, 1);
  PS4.begin("b8:d6:1a:67:70:22");
  ps4SetEventCallback(controller_event_cb);
  SerialBT.begin("ESP32test");
  
  delay(100);
  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
 
    Locomote();
 
}

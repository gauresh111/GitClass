#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11);  // RX | TX
char shortest[1000]={};
int arrayOfPins[] = {
 38,39,40,41,42,43,44,45
};
//motor

int dig1 = 2;
int dig2 = 3;
int an1 = 4;
int an2 = 5;
int PWM = 200;
int left_pwm = PWM;
int right_pwm = PWM;
int sensitivity = 60;
//pid
double Kp = 0.42;  //proportional gain
double Ki = 0;  //integral gain
double Kd =0;  //derivative gain
int setPoint = 998/2;
double total_error, last_error;
float lastError = 0;
float error;
//lsa-08
int lsaInputs[8];
bool lsa_work = true;
char address = 0x01;  // UART address as 0x01
int analog_value = A5;
int position;
int previous_position;
bool print = true;
bool line_mode_value = false;
bool is_stop=false;
int junction_pin=7;
int junction;
void PidControl(float positionVal) {

  float error = positionVal - setPoint;                                          // Calculate the deviation from position to the set point
  float motorSpeed = Kp * error + Kd * (error - lastError) + Ki * (Ki + error);  // Applying formula of PID
  lastError = error;                                                             // Store current error as previous error for next iteration use

  // Adjust the motor speed based on calculated value
  // You might need to interchange the + and - sign if your robot move in opposite direction
  float rightMotorSpeed = PWM - motorSpeed;
  float leftMotorSpeed = PWM + motorSpeed;

  // If the speed of motor exceed max speed, set the speed to max speed
  if (rightMotorSpeed > 255) rightMotorSpeed = 255;
  if (leftMotorSpeed > 255) leftMotorSpeed = 255;

  // If the speed of motor is negative, set it to 0
  if (rightMotorSpeed < 75) rightMotorSpeed =0;
  if (leftMotorSpeed < 75) leftMotorSpeed = 0;
  right_pwm = leftMotorSpeed;
  left_pwm = rightMotorSpeed;
  // Serial.print("Right Motor:%f , Left Motor: %f   positionVal:%f \n ", right_pwm, left_pwm,positionVal);
  Serial.print("RIGHT MOtor:");
  Serial.print(right_pwm);
  Serial.print("  LEFT MOTOR:");
  Serial.print(left_pwm);
  Serial.print(" Position:");
  Serial.println(positionVal);
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
  clockwise(255, 255);
  delay(5000);
  stop();
}

void printLSAValues() {
  for (int i = 0; i < 8; i++) {
    Serial.print((String) "ir" + i + ":");
    Serial.print(lsaInputs[i]);
    Serial.print(" ");
  }
  Serial.print("  ");
  Serial.print("previous_position: ");
  Serial.print(previous_position);
  Serial.print("  ");
  Serial.print(" Analog Value: ");
  Serial.print(position);
    Serial.print("  ");
  Serial.print(" junction: ");
  Serial.println(junction);
}

void readLSAInputs() {
  for (int i = 0; i < 8; i++) {
    lsaInputs[i] = digitalRead(arrayOfPins[i]);
  }
  position = analogRead(analog_value);
  junction=digitalRead(junction_pin);
}

void setup_pinMODE() {
  pinMode(an1, INPUT);
  pinMode(an2, INPUT);
  pinMode(dig1, OUTPUT);
  pinMode(dig2, OUTPUT);
  pinMode(analog_value, INPUT);
  pinMode(junction_pin, INPUT);
  for (int i = 0; i < 8; i++) {
    pinMode(arrayOfPins[i], INPUT);
  }
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
///  Serial.println("### --- forward");
}
void left(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, LOW);
  digitalWrite(dig2, HIGH);
  Serial.println("### --- left");
}
void right(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, HIGH);
  digitalWrite(dig2, LOW);
  Serial.println("### --- right");
}
void clockwise(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, HIGH);
  digitalWrite(dig2, LOW);
  Serial.println("### --- Clockwise");
}
void anti_clockwise(int left, int right) {
  analogWrite(an1, left);
  analogWrite(an2, right);
  digitalWrite(dig1, LOW);
  digitalWrite(dig2, HIGH);
  Serial.println("### --- AntiClockwise");
}
void goOneInch() {
  forward(200, 200);
  Serial.println(" Now Move Forward");
  delay(1000);
  Serial.println("Now Stop");
  stop();
}



void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  BTSerial.begin(38400);
  setup_pinMODE();
  delay(50);
  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  readLSAInputs();
  // printLSAValues();
  if (print) {
    pid_print();
  }
  pid_change();
  if (line_mode_value) {
    line_Mode();
  }
 //line_Mode();
}

void pid_change() {
  if (BTSerial.available()) {
    print = false;
    char character = "";
    String data_from_display = "";
    delay(30);

    while (BTSerial.available()) {
      character = char(BTSerial.read());
      data_from_display += character;
    }

    String(value) = data_from_display;
    if (data_from_display[0] == 'p') {
      value.remove(0, 1);
      Kp = value.toFloat();
    } else if (data_from_display[0] == 'i') {
      value.remove(0, 1);
      Ki = value.toFloat();
    } else if (data_from_display[0] == 'd') {
      value.remove(0, 1);
      Kd = value.toFloat();
    } else if (data_from_display[0] == 'a') {
      print = true;
    } else if (data_from_display[0] == 's') {
      stop();
      line_mode_value = false;
       
    } else if (data_from_display[0] == 'f') {
      line_mode_value = true;
    }
    else if(data_from_display[0]=='c')
    {
      calibrate();
    }
    else if (data_from_display[0] == 'q') {
      value.remove(0, 1);
      PWM = value.toFloat();
    }
    } 
  }

void pid_print() {
  BTSerial.print("Kp :");
  BTSerial.print(Kp);
  BTSerial.print("     Ki :");
  BTSerial.print(Ki);
  BTSerial.print("     Kd :");
  BTSerial.print(Kd);
  BTSerial.print(" analog :");
  BTSerial.println(previous_position);
}

void line_Mode() {
  if (lsaInputs[1] == 1 || lsaInputs[2] == 1 || lsaInputs[3] == 1 || lsaInputs[4] == 1 || lsaInputs[5] == 1 || lsaInputs[6] == 1) {
    PidControl(position);
    forward(left_pwm, right_pwm);
    previous_position=position;
  } else {
    if(previous_position>setPoint)
    {
      previous_position=1024;
    }
    else {
      previous_position=0;
    }
    PidControl(previous_position);
    forward(left_pwm, right_pwm);
  }

}

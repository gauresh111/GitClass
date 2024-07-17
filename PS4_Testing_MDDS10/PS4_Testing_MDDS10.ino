#include <PID_v1.h>
#include <PS4Controller.h>

int anL1 = 16;  //13
int digL1 = 5;
int anL2 = 4;  //13
int digL2 = 17;

int sensitivity = 60;
int isLocked = 0;
int input;
int stop = 100;
int is_diagonal = 0;
int pwm = 50;
int RPM = 5;


void onConnect() {
  Serial.println("Connected");
}

void onDisConnect() {
  Serial.println("Disconnected");
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
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ps4SetEventCallback(controller_event_cb);
  // put your setup code here, to run once:
  pinMode(anL1, INPUT);
  pinMode(digL1, OUTPUT);
  pinMode(anL2, INPUT);
  pinMode(digL2, OUTPUT);
  Stop();
  delay(2000);

  PS4.attachOnConnect(onConnect);
  PS4.begin("b8:d6:1a:67:70:22");  // 7c:9e:bd:47:94:06 joel // b8:d6:1a:67:70:20 nathan
}

void loop() {
  // put your main code here, to run repeatedly:
  forward(20);
  delay(1000);
  reverse(20);
  delay(1000);
  // if (isLocked == false) {
  //     Locomote();
  //   } else {
  //     Stop();
  //   }
}
void forward(int rpm) {
  //getRPM();
  is_diagonal = 0;
  analogWrite(anL1, pwm);
  analogWrite(anL2, pwm);
  // analogWrite(anR1, pwm);
  // analogWrite(anR2, pwm);
  digitalWrite(digL1, HIGH);
  digitalWrite(digL2, HIGH);
  // digitalWrite(digR1, HIGH);
  // digitalWrite(digR2, HIGH);
}

void reverse(int rpm) {
  //getRPM();
  is_diagonal = 0;

  analogWrite(anL1, pwm);
  analogWrite(anL2, pwm);
  // analogWrite(anR1, pwm3);
  // analogWrite(anR2, pwm4);
  digitalWrite(digL1, LOW);
  digitalWrite(digL2, LOW);
  // digitalWrite(digR1, LOW);
  // digitalWrite(digR2, LOW);
}


void Stop() {
  //getRPM();

  digitalWrite(digL1, 0);
  digitalWrite(digL2, 0);
  // digitalWrite(digR1, [2]);
  // digitalWrite(digR2, [3]);

  analogWrite(anL1, 0);
  analogWrite(anL2, 0);
  // analogWrite(anR1, pwm3);
  // analogWrite(anR2, pwm4);
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


 if (lyAxis < -sensitivity && abs(lxAxis) < 50) {

    reverse(mlyAxis);
    Serial.println("Reverse");
  }

  else if (lyAxis > sensitivity && abs(lxAxis) < 50) {
    forward(mlyAxis);
    Serial.println("Forward");
  }

  else {

    Stop();
  }
}



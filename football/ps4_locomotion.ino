#include <PS4Controller.h>
int anR = 18;
int digR =16 ;
int anL =  5;//13
int digL = 17;
int Setpoint=100;
int sensitivity=60;
void controller_event_cb(ps4_t ps4, ps4_event_t event) {
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
void pin_Mode()
{
 pinMode(anR, INPUT);
  pinMode(digR, OUTPUT);
  pinMode(anL, INPUT);
  pinMode(digL, OUTPUT); 
}
void Forward(){
digitalWrite(digR, LOW);
digitalWrite(digL, LOW);
analogWrite(anL, Setpoint); 
analogWrite(anR, 0 );
Serial.print("FORWARD  ");
Serial.println(Setpoint);

}
void Reverse(){
digitalWrite(digR, LOW);
digitalWrite(digL, HIGH);
analogWrite(anL, Setpoint );
analogWrite(anR, Setpoint );
Serial.print("REVERSE  ");
Serial.println(Setpoint);
}
void Left()
{


digitalWrite(digR, HIGH);
digitalWrite(digL, LOW);
analogWrite(anL, Setpoint );
analogWrite(anR, Setpoint );
Serial.print("LEFT  ");
Serial.println(Setpoint);

}
void Right()
{

digitalWrite(digR, LOW);
digitalWrite(digL, HIGH);
analogWrite(anL, Setpoint );
analogWrite(anR, Setpoint );
Serial.print("RIGHT  ");
Serial.println(Setpoint);
}
void Stop()
{

digitalWrite(digR, HIGH);
digitalWrite(digL, HIGH);
analogWrite(anL, 0 );
analogWrite(anR, 0);
Serial.println("STOP");
}
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
	pin_Mode()
  ps4SetEventCallback(controller_event_cb);
   PS4.begin("b8:d6:1a:67:70:33"); 
    Stop();
    delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:
Locomote();
}
void Locomote()
{
  int lxAxis = PS4.LStickX();
  int lyAxis = PS4.LStickY() * (1);
  int rxAxis = PS4.RStickX();
  int ryAxis = PS4.RStickY() * (1);

  if (lyAxis < -sensitivity && abs(lxAxis) < 50) {
  
    Reverse();
  }

  else if (lyAxis > sensitivity && abs(lxAxis) < 50) {
 
    Forward();
  }
  else if (rxAxis > sensitivity && abs(ryAxis) < 50) {
  
    Right();
  }

  else if (rxAxis < -sensitivity && abs(ryAxis) < 50) {
   Left();
  }
  else{
    Stop();
  }

}

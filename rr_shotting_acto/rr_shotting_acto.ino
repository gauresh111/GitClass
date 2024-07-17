
#include "Wire.h"



#define I2C_DEV_ADDR 0x50

int ledState = LOW;  // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change :
const long interval = 100;

int acuator_digital = 5;
int acuator_analog = 17;
int shotter_analog = 18;
int shotter_digital = 19;
int ir1 = 14;
int ir2 = 13;
int pole_1 = 0;
int pole_2 = 0;
int target = 0;
const int trigPin = 4;
const int echoPin = 15;
const int ledPin = 33;
int isShareON = false;
void onReceive(int len) {
  // Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    int value = Wire.read();
    if (value == 49) {
      move_shooter(255, 1);
      Serial.println("shooter _ up");
    } else if (value == 50) {
      move_shooter(255, 0);
      Serial.println("shooter _ down");
    } else if (value == 51) {
      move_shooter(0, 0);
    } else if (value == 52) {
      move_acuator(255, 0);
      Serial.println("acuator _ up");
    } else if (value == 53) {
      move_acuator(255, 1);
      Serial.println("acuator _ down");
    } else if (value == 54) {
      move_acuator(0, 0);
    } else if (value == 55) {
      target = 1;
    } else if (value == 56) {
      target = 2;
    } else if (value == 58) {
      target = 0;
    } else if (value == 57) {
      if (target != 0) {
        if (target == 1) {

          while (pole_1 != 0) {
            pole_1 = digitalRead(ir1);
            pole_2 = digitalRead(ir2);
            move_shooter(255, 1);
          }
          move_shooter(0, 1);
        }
        if (target == 2) {
          while (pole_2 != 0) {
            pole_1 = digitalRead(ir1);
            pole_2 = digitalRead(ir2);
            move_shooter(255, 1);
          }
          move_shooter(0, 1);
        }
      }
    }
    if (value == 97) {
      if (isShareON == false) {
        isShareON = true;
      } else {
        isShareON = false;
      }
    }
    Serial.println(value);
    Serial.println(target);
  }

}

  void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.setDebugOutput(true);
    Wire.onReceive(onReceive);
    Wire.begin((uint8_t)I2C_DEV_ADDR);
   // xTaskCreatePinnedToCore(distance, "Task1", 10000, NULL, 1, NULL, 1);
    pinMode(shotter_digital, OUTPUT);
    pinMode(shotter_analog, INPUT);
    pinMode(acuator_analog, INPUT);
    pinMode(acuator_digital, OUTPUT);
    pinMode(ir1, INPUT);
    pinMode(ir2, INPUT);
    pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);
    pinMode(ledPin, OUTPUT);
    move_shooter(0, 0);
    move_acuator(0, 0);

    delay(100);
  }

  void loop() {
    // put your main code here, to run repeatedly:
    // pole_1 = digitalRead(ir1);
    // pole_2 = digitalRead(ir2);
    //  move_shooter(255, 1);
    //  delay(1000);
    //   move_shooter(255, 0);
    // Serial.println((String) "pole_1: " + pole_1 + " pole_2: " + pole_2);

     if (isShareON == true) {
      ledConditions(getUltraSonicDistance());
      //Serial.print("uktra");
    }
  }
  void move_shooter(int speed, int direction) {
    analogWrite(shotter_analog, speed);
    digitalWrite(shotter_digital, direction);
  }
  void move_acuator(int speed, int direction) {
    analogWrite(acuator_analog, speed);
    digitalWrite(acuator_digital, direction);
  }



  //

  int getUltraSonicDistance() {
    int count = 0;

    int sum = 0;
    long duration, inches, cm;
    pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    //inches = microsecondsToInches(duration);
    cm = microsecondsToCentimeters(duration);
    Serial.println(cm);
    return cm;
  }

  void ledConditions(int cm) {
    if (cm < 40) {
      digitalWrite(ledPin, 1);
    } else if (cm > 42 && cm < 48) {
      digitalWrite(ledPin, 0);
    } else if (cm >= 48 && cm < 60) {
      blink_led(10);
    } else if (cm >= 61 && cm < 70) {
      blink_led(100);
    } else if (cm >= 71 && cm < 80) {
      blink_led(250);
    } else {
      digitalWrite(ledPin, 1);
    }
  }
  void blink_led(int inter) {
    unsigned long currentMillis = millis();
    int interval = inter;
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    }
  }
  long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2;
  }

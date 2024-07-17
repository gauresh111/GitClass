#include <Arduino.h>


#include "Wire.h"
#include "WiFi.h"
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include <ArduinoJson.h>
#include "Wire.h"
const char *ssid = "ESP32-Access-Point";
const char *password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
byte dataArray[2];
AsyncWebSocketClient *globalClient = NULL;

const char index_html[] PROGMEM = R"rawliteral(
        <!DOCTYPE html>
<html>
   <head>
      <script type = "text/javascript">
        var ws = new WebSocket("ws://192.168.4.1/ws");
 
        ws.onopen = function() {
            window.alert("Connected");
         };
 
         ws.onmessage = function(evt) {
           const obj =  JSON.parse(evt.data);
            console.log(obj);           
            document.getElementById("display").innerHTML  = "distance: " + obj.value  + " CM";
            if(obj.status==1){
              document.getElementById("status").innerHTML  = "ultrasonic is on";
            }
            if(obj.status==0){
              document.getElementById("status").innerHTML  = "ultrasonic is off";
            }
        };
 
      </script>
   </head>
 
   <body>
      <div  style="text-align:center; font-size: 30px;" >
         <h1 id = "display" >Not connected</h1>
         <h1 id="status"></h1>        
      </div>

   </body>
</html>
)rawliteral";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {

    Serial.println("Websocket client connection received");
    globalClient = client;

  } else if (type == WS_EVT_DISCONNECT) {

    Serial.println("Websocket client connection finished");
    globalClient = NULL;
  }
}


#define I2C_DEV_ADDR 0x50
int ledState = LOW;  // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change :

const long interval = 100;
int shotter_digital = 19;
int shotter_analog = 18;
int acuator_analog = 17;
int acuator_digital = 5;
int ir1 = 14;
int ir2 = 13;
int pole_1 = 0;
int pole_2 = 0;
int target = 0;
int recall_shooter=25;
int shooter_re=0;
int status=0;
const int trigPin = 4;
const int echoPin = 15;
const int ledPin = 33;
int isShareON = false;
int disat=0;
int delayss;
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
void loco(void* pvParameters) {

  Serial.print("locomotion running on core ");

  Serial.println(xPortGetCoreID());

  for (;;) {
    if (isShareON == true) {
    if(delayss==1)
    {
        digitalWrite(ledPin, 0);
       // Serial.println("1");

    }
    else if ( delayss==3)
    {
        digitalWrite(ledPin, 1);
    }
    else if (delayss==4){
      blink_led(5000);
    }
    else{
    blink_led(delayss);
  }
  }
  else {
    digitalWrite(ledPin, 1);
  }
  }
}
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
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
  disat=cm;
  return cm;
}

void serve_distance(void* pvParameters)
{
   Serial.print("locomotion running on core ");

  Serial.println(xPortGetCoreID());

  for (;;) {
   
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


void ledConditions(int cm,int target) {
 if(target==1){
  if (cm < 44 && cm !=0) {
  
    delayss = 3;
  } else if (cm >44 && cm <=48 ) {
  
    delayss =1 ;
  } else if (cm >= 49 && cm < 60) {
    delayss=100;
    //blink_led(50);
  } else if (cm >= 61 && cm < 70) {
    //blink_led(200);
    delayss=200;
  } else if (cm >= 71 && cm < 80) {
    //blink_led(500);
    delayss=500;
  } else {
    // digitalWrite(ledPin, 0);
    delayss =4;
  }
 }
 else if (target==2)
 {
   if (cm < 93 && cm !=0) {
  
    delayss = 3;
  } else if (cm > 92 && cm <=97 ) {
  
    delayss =1 ;
  } else if (cm >= 98 && cm < 106) {
    delayss=100;
    //blink_led(50);
  } else if (cm >= 107 && cm < 117 ) {
    //blink_led(200);
    delayss=200;
  } else if (cm >= 118 && cm < 128) {
    //blink_led(500);
    delayss=500;
  } else {
    // digitalWrite(ledPin, 0);
    delayss =4;
  }
 }
}


void from_esp(int number) {
  switch (number) {
    case 49:
  
      {
        move_shooter(255, 1);
        Serial.println("shooter _ up");
        break;
      }
    case 50:
      {
        move_shooter(255, 0);
        Serial.println("shooter _ down");
        break;
      }
    case 51:
      {
        move_shooter(0, 0);
              Serial.println("shooter _ stop");
        break;
      }
    case 52:
      {
        move_acuator(150, 0);
        Serial.println("acuator _ up");
        break;
      }
    case 53:
      {
        move_acuator(150, 1);
        Serial.println("acuator _ down");
        break;
      }
    case 54:
      {
        move_acuator(0, 0);
        Serial.println("acuator _ stop");
        break;
      }
    case 55:
      {
        target = 1;
        break;
      }
    case 56:
      {
        target = 2;
        break;
      }
    case 57:
      {
        if (target != 0) {
          if (target == 1) {

            while (pole_1 != 0) {
              pole_1 = digitalRead(ir1);
              pole_2 = digitalRead(ir2);
              move_shooter(230, 1);
            }
            move_shooter(0, 1);
          }
          if (target == 2) {
            while (pole_2 != 0) {
              pole_1 = digitalRead(ir1);
              pole_2 = digitalRead(ir2);
              move_shooter(230, 1);
            }
            move_shooter(0, 1);
          }
          break;
        }
      }
    case 58:
      {
        target = 0;
        break;
      }
    case 97:
      {
        if (isShareON == false) {
          isShareON = true;
        } else {
          isShareON = false;
        }
        break;
      }
    case 98:
    {
      while(shooter_re!=0)
      {
        shooter_re=digitalRead(recall_shooter);
         move_shooter(230, 0);
      }
       move_shooter(0, 1);
    }
  }
}

void onReceive(int len) {
  // Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    int value = Wire.read();

    from_esp(value);
    // Serial.println(value);
    // Serial.println(target);
  }
  // Serial.println();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  xTaskCreatePinnedToCore(loco, "Task1", 10000, NULL, 0, NULL, 1);
  xTaskCreatePinnedToCore(serve_distance, "Task1", 10000, NULL, 0, NULL, 0);
  Wire.onReceive(onReceive);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

  pinMode(shotter_digital, OUTPUT);
  pinMode(shotter_analog, INPUT);
  pinMode(acuator_analog, INPUT);
  pinMode(acuator_digital, OUTPUT);
  pinMode(recall_shooter,INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  move_shooter(0, 0);
  move_acuator(0, 0);
   WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);


  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });
  server.on("/html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html);
  });
    server.begin();
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
 pole_1 = digitalRead(ir1);
  pole_2 = digitalRead(ir2);
  shooter_re=digitalRead(recall_shooter);
  //  move_shooter(255, 1);
  //  delay(1000);
  //   move_shooter(255, 0);
 // Serial.println((String) "shooter_re: " + shooter_re + " pole_2: " + pole_2);
  if (isShareON == true) {
   ledConditions(getUltraSonicDistance(),target);
   status=1;
    //Serial.print("uktra");
  }
  else {
    status =0;
  }
   if (globalClient != NULL && globalClient->status() == WS_CONNECTED) {
    


    // Create a JSON object with a maximum size of 200 bytes
    StaticJsonDocument<200> json;

    // Add some data to the JSON object
    //json["Status"] = status;
    json["value"] = disat;
    json["status"]= status;
    String jsonString;
    serializeJson(json, jsonString);
    // Convert variables to JSON format
    //Serial.println(jsonString);

    globalClient->text(jsonString);
    delay(50);
  }
  
}

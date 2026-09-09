// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html 
#include <ESP32Servo.h>
#define SERVO_PIN 18

static const int servoPin = 32;


/*
void setup() {
  Serial.begin(115200);

  //ledcSetup(0, 50, 16);   // Canal 0, 50 Hz, 16 bits
  //ledcAttachPin(SERVO_PIN, 0);
  ledcAttachChannel(SERVO_PIN, 50, 16, 0);
}
*/

Servo servo1;

void setup() {

  Serial.begin(115200);
  servo1.attach(servoPin);
}

/*
bool setAngle(int angle) {
  // 0-180° → 1ms-2ms
  int duty = map(angle, 0, 180, 3277, 6553); 
  // 16 bits, période 20ms → 65535 max
  bool r= ledcWrite(0, duty);
  return r;
}

void loop() {
  if (Serial.available()) {
    Serial.println("commmand received !");
    int angle = Serial.parseInt();
    bool r=setAngle(angle);
    Serial.println(r);
  }
}

*/

void loop() {
  for(float posDegrees = 60; posDegrees <= 120; posDegrees+=0.5) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(10);
  }

  for(float posDegrees = 120; posDegrees >= 60; posDegrees-=0.5) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(10);
  }
 
}

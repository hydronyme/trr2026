// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html 


#define RPWM 25
#define LPWM 26
#define REN  27
#define LEN  14

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
}
/*
const int pwmPin = RPWM;
const int pwmChannel = 0;
const int pwmFreq = 20000;
const int pwmResolution = 8;

void setup() {

  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);
  ledcAttach(pwmPin, pwmFreq, pwmResolution);

}

void loop() {

  digitalWrite(REN, HIGH);
  for(int duty = 255; duty >= 0; duty--) {
    ledcWrite(pwmChannel, duty);
    delay(10);
  }

}
*/
void loop() {
  // Avancer
  analogWrite(RPWM, 50);
  analogWrite(LPWM, 0);
  delay(3000);

  // Stop
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  delay(2000);

  // Reculer
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 50);
  delay(3000);
}


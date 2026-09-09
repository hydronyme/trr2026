// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html 
#include <ESP32Servo.h>
#define SERVO_PIN 32 
#define RPWM 25
#define LPWM 26
#define REN  27
#define LEN  14


Servo servo1;

void setup() {
  Serial.begin(115200);
  servo1.attach(SERVO_PIN);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
}

void setAngle(int angle) {
    Serial.print("Commande ANGLE:");
    Serial.println(angle);
    int alpha = 90+ 10 + angle;
    servo1.write(alpha);
}

void setForwardSpeed(int speed) {
  Serial.print("Commande FORWARD:");
  Serial.println(speed);
  analogWrite(RPWM, speed);
  analogWrite(LPWM, 0);
}

void stop() {
  Serial.println("Commande STOP");
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void setBackwardSpeed(int speed) {
  Serial.print("Commande BACK:");
  Serial.println(speed);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, speed);
}

void loop() {
	if (Serial.available()) {
		Serial.print("commmand received ->");
		//String cmd = "LED:1";
		String cmd = Serial.readStringUntil('\n');
		Serial.println(cmd);
		int separatorIndex = cmd.indexOf(':');

		if (separatorIndex != -1) {
			String commande = cmd.substring(0, separatorIndex);
			String valeur = cmd.substring(separatorIndex + 1);

			if(commande=="forward") {
				int speed = valeur.toInt();
				setForwardSpeed(speed);
			}
			if(commande=="backward") {
				int speed = valeur.toInt();
				setBackwardSpeed(speed);
			}
			if(commande=="stop") {
				stop();
			}
			if(commande=="angle") {
				int angle = valeur.toInt();
				setAngle(angle);
			}
		}

	}
}


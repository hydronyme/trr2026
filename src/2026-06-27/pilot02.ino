// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html 
#include <ESP32Servo.h>

#define SERVO_PIN 32

#define RPWM 25
#define LPWM 26
#define REN  27
#define LEN  14

// Encodeur
#define ENCODER_A 18
#define ENCODER_B 19

Servo servo1;

// ---------- PWM ----------
#define PWM_CHANNEL_R 0
#define PWM_CHANNEL_L 1
#define PWM_FREQ      20000
#define PWM_RESOLUTION 8

// ---------- Encodeur ----------
volatile long encoderCount = 0;

// Adapter selon ton encodeur
// Exemple : 200 P/R en x2
const int COUNTS_PER_REV = 600;

// ---------- Régulation ----------
float targetRPM = 0;
float currentRPM = 0;

float Kp = 0.2;

int pwmOutput = 0;
float filteredRPM = 0;

long lastCount = 0;
unsigned long lastTime = 0;

// ---------- Sens ----------
bool forwardDirection = true;
bool stop_mode = true;

// ======================================================
// INTERRUPTION ENCODEUR
// ======================================================

void IRAM_ATTR handleEncoder()
{
    bool a = digitalRead(ENCODER_A);
    bool b = digitalRead(ENCODER_B);

    if (a != b)
        encoderCount++;
    else
        encoderCount--;
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
    Serial.begin(921600);

    // Servo
    servo1.attach(SERVO_PIN);

    // Enable moteur
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);

    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);

    // PWM hardware ESP32
    ledcAttach(RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LPWM, PWM_FREQ, PWM_RESOLUTION);

    // Encodeur
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoder, CHANGE);
    lastTime = millis();
}

// ======================================================
// DIRECTION
// ======================================================

void setAngle(int angle)
{
    uint32_t t0 = micros();
    int alpha = 90 + 10 + angle;
    servo1.write(alpha);
    uint32_t t1 = micros();
    Serial.print("ANGLE");
    Serial.println(angle);
    Serial.print("temps ANGLE");
    Serial.println(t1 - t0);
}

// ======================================================
// CONSIGNE VITESSE
// ======================================================

void setForwardRPM(float rpm)
{
    targetRPM = rpm;
    forwardDirection = true;
    stop_mode = false;
    Serial.println("FORWARD");
}

void setBackwardRPM(float rpm)
{
    targetRPM = rpm;
    forwardDirection = false;
    stop_mode = false;
    Serial.println("BACKWARD");
}

void stopMotor()
{
    targetRPM = 0;

    ledcWrite(RPWM, 0);
    ledcWrite(LPWM, 0);
    stop_mode = true;
    Serial.println("STOP");
}

// ======================================================
// BOUCLE PID
// ======================================================

void updateMotorControl()
{
    unsigned long now = millis();

    // toutes les 10 ms
    if (now - lastTime >= 10 && !stop_mode)
    {
        noInterrupts();
        long count = encoderCount;
        interrupts();

        long delta = count - lastCount;
        //Serial.print(" COUNT=");
        //Serial.println(count);
        // RPM
        currentRPM = delta;

        if (!forwardDirection) currentRPM = -currentRPM;

	//filteredRPM = 0.8 * filteredRPM + 0.2 * currentRPM;

	//float correction = 0.5 * (targetRPM - filteredRPM);
	float correction = (targetRPM - currentRPM);
	correction = constrain(correction, -1, 1);
	pwmOutput += correction;
	pwmOutput = constrain(pwmOutput, 0, 150);

        // appliquer PWM
        if (forwardDirection)
        {
            ledcWrite(RPWM, pwmOutput);
            ledcWrite(LPWM, 0);
        }
        else
        {
            ledcWrite(RPWM, 0);
            ledcWrite(LPWM, pwmOutput);
        }

        // debug
        Serial.print("RPM=");
        Serial.print(currentRPM);

        Serial.print(" TARGET=");
        Serial.print(targetRPM);

        Serial.print(" PWM=");
        Serial.println(pwmOutput);

        lastCount = count;
        lastTime = now;
    }
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
    updateMotorControl();

    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        int separatorIndex = cmd.indexOf(':');
        if (separatorIndex != -1)
        {
            String commande = cmd.substring(0, separatorIndex);
            String valeur = cmd.substring(separatorIndex + 1);

            if (commande == "angle") {
                int angle = valeur.toInt();
                setAngle(angle);
            }

            if (commande == "forward") {
                float rpm = valeur.toFloat();
                stop_mode = false;
                setForwardRPM(rpm);
            }

            if (commande == "backward") {
                float rpm = valeur.toFloat();
                stop_mode = false;
                setBackwardRPM(rpm);
            }


	    if (commande == "stop") {
		stopMotor();
	    }
        }
    }
}

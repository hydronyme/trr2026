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
float targetRPM = 0.;
float currentRPM = 0.;

float Kp = 0.2;

int pwmOutput = 0;
float filteredRPM = 0;

long lastCount = 0;
uint32_t lastTime = 0;

// ---------- Sens ----------
bool forwardDirection = true;
bool stop_mode = true;

typedef struct __attribute__((packed)) {
    uint32_t time;
    uint32_t lastTime;
    long count;
    long lastCount;
    float currentRPM;
    float targetRPM;
    int lastPwm;
    int currentPwm;
} imu_packet_t;

imu_packet_t pkt;

typedef struct __attribute__((packed)) {
  uint16_t command;
  int value;
} cmd_packet_t;

cmd_packet_t cmd_pkt;
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
    int alpha = 90 + 10 + angle;
    servo1.write(alpha);
}

// ======================================================
// CONSIGNE VITESSE
// ======================================================

void setForwardRPM(int rpm)
{
    targetRPM = rpm;
    forwardDirection = true;
    stop_mode = false;
}

void setBackwardRPM(int rpm)
{
    targetRPM = rpm;
    forwardDirection = false;
    stop_mode = false;
}

void stopMotor()
{
    targetRPM = 0;

    ledcWrite(RPWM, 0);
    ledcWrite(LPWM, 0);
    stop_mode = true;
}

// ======================================================
// BOUCLE PID
// ======================================================

void updateMotorControl()
{
    uint32_t now = millis();

    // toutes les 100 ms
    if (now - lastTime >= 50 && !stop_mode)
    {
        noInterrupts();
        long count = encoderCount;
        long delta = count - lastCount;
        currentRPM = delta;

        pkt.time = now;
        pkt.lastTime = lastTime;
        pkt.count = count;
        pkt.lastCount = lastCount;
        pkt.currentRPM = currentRPM;
        pkt.targetRPM = targetRPM;
        pkt.lastPwm = pwmOutput;

        if (!forwardDirection) currentRPM = -currentRPM;

        //filteredRPM = 0.8 * filteredRPM + 0.2 * currentRPM;

        if (currentRPM == 0) {
            pwmOutput += 1;
            pwmOutput = constrain(pwmOutput, 0, 100);
        }
        else {
            //float correction = 0.5 * (targetRPM - filteredRPM);
            float correction = (targetRPM - currentRPM);
            correction = constrain(correction, -1, 1);
            pwmOutput += correction;
            pwmOutput = constrain(pwmOutput, 0, 100);
        }
        pkt.currentPwm = pwmOutput;

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

        //Serial.write((uint8_t*)&pkt, sizeof(pkt));

        lastCount = count;
        lastTime = now;
        interrupts();
    }
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  updateMotorControl();

  if (Serial.available() >= sizeof(cmd_pkt)) {

    Serial.readBytes((char*)&cmd_pkt, sizeof(cmd_pkt));

    int command = cmd_pkt.command;
    int valeur = cmd_pkt.value;

    if (command == 3) setAngle(valeur);
    else if (command == 0) stopMotor();
    else if (command == 1) setForwardRPM(valeur);
    else if (command == 2) setBackwardRPM(valeur);
  }
}

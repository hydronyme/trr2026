#define ENCODER_A 18
#define ENCODER_B 19

#define MOTOR_PWM 25

// Résolution encodeur
// Exemple : 200 P/R
// Ici lecture x2 => 400 counts/tour
const int COUNTS_PER_REV = 400;

// Consigne vitesse
float targetRPM = 120.0;

// Gain proportionnel
float Kp = 1.5;

// Compteur encodeur
volatile long encoderCount = 0;

// Variables RPM
long lastCount = 0;
unsigned long lastTime = 0;

// PWM
int pwmValue = 0;

// ----- INTERRUPTION ENCODEUR -----

void IRAM_ATTR handleEncoder()
{
    bool a = digitalRead(ENCODER_A);
    bool b = digitalRead(ENCODER_B);

    if (a == b)
        encoderCount++;
    else
        encoderCount--;
}

void setup()
{
    Serial.begin(115200);

    // Encodeur
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A),
        handleEncoder,
        CHANGE);

    // PWM ESP32
    ledcSetup(0, 20000, 8); // canal 0, 20 kHz, 8 bits
    ledcAttachPin(MOTOR_PWM, 0);

    lastTime = millis();
}

void loop()
{
    unsigned long now = millis();

    // Calcul toutes les 100 ms
    if (now - lastTime >= 100)
    {
        noInterrupts();
        long count = encoderCount;
        interrupts();

        long delta = count - lastCount;

        // RPM
        float rpm =
            (delta * 600.0) / COUNTS_PER_REV;

        // Erreur
        float error = targetRPM - rpm;

        // Contrôle proportionnel
        pwmValue += Kp * error;

        // Limites PWM
        pwmValue = constrain(pwmValue, 0, 255);

        // Sortie PWM
        ledcWrite(0, pwmValue);

        // Debug
        Serial.print("RPM: ");
        Serial.print(rpm);

        Serial.print("  PWM: ");
        Serial.print(pwmValue);

        Serial.print("  Count: ");
        Serial.println(count);

        lastCount = count;
        lastTime = now;
    }
}

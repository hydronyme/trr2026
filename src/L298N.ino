// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/ledc.html 


// Motor control pins
#define ENA 32
#define IN1 21
#define IN2 19


// PWM settings
#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8  // 0–255

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Setup PWM
  //ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  //ledcAttachPin(ENA, PWM_CHANNEL);

  Serial.begin(115200);
}

void loop() {
  // Motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //ledcWrite(PWM_CHANNEL, 150); // speed (0–255)
  analogWrite(ENA,255);
  Serial.println("Motor forward");
  delay(3000);

  // Stop motor
  //ledcWrite(PWM_CHANNEL, 0);
  analogWrite(ENA,0);
  Serial.println("Motor stop");
  delay(2000);

  // Motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA,255);
  //ledcWrite(PWM_CHANNEL, 150);
  Serial.println("Motor backward");
  delay(3000);

  // Stop again
  analogWrite(ENA,0);
  //ledcWrite(PWM_CHANNEL, 0);
  delay(2000);
}


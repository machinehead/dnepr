#include <Servo.h>

#define LED  (13)

Servo pwm1, pwm2;

#define PWM1_PIN (3)
#define PWM2_PIN (5)

#define ESC_DISARM (900)

#define THROTTLE_MIN (1150)
#define THROTTLE_MAX (1850)

#define dt 0.05

float alpha = dt / (1.0/(2 * 3.14159265 * 1) + dt);

void writeMotors(int throttle)
{
  pwm1.writeMicroseconds(throttle);
  pwm2.writeMicroseconds(throttle);
}

void setup() {                
  Serial.begin(115200);
  // initialize the LED pin as an output.  
  pinMode(LED, OUTPUT);
  pwm1.attach(PWM1_PIN);
  pwm2.attach(PWM2_PIN);
  digitalWrite(LED, HIGH);
// writeMotors(THROTTLE_MAX);
// delay(3000);
  writeMotors(ESC_DISARM);
  digitalWrite(LED, LOW);
  delay(20);
}

// the loop routine runs over and over again forever:
void loop() {
  static unsigned int ledState = LOW;
  static unsigned long lastLedTime = millis();
  static int sensorRaw = 0, sensorValue = 0;
  if( millis() - lastLedTime > 50 )
  {
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(LED, ledState);
    lastLedTime = millis();
    sensorRaw = analogRead(A0);
    sensorValue = alpha * ((float)sensorRaw) + (1.0 - alpha)*((float)sensorValue);
  }

  int throttle = map(sensorValue, 0, 1023, ESC_DISARM, THROTTLE_MAX);
  writeMotors(throttle);
  Serial.println(throttle);
}

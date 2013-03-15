#include <Servo.h>

#define LED  (13)

Servo pwm1, pwm2;

#define PWM1_PIN (3)
#define PWM2_PIN (5)

#define ESC_DISARM (1000)

#define THROTTLE_MIN (1150)
#define THROTTLE_MAX (1850)

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
  writeMotors(THROTTLE_MAX);
  delay(2000);
  writeMotors(ESC_DISARM);
  digitalWrite(LED, LOW);
  delay(20);
}

// the loop routine runs over and over again forever:
void loop() {
  static unsigned int ledState = LOW;
  static unsigned long lastLedTime = millis();
  if( millis() - lastLedTime > 100 )
  {
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(LED, ledState);
    lastLedTime = millis();
  }
    
  int sensorValue = analogRead(A0);
  int throttle = map(sensorValue, 0, 1023, ESC_DISARM, THROTTLE_MAX);
  writeMotors(throttle);
  Serial.println(throttle);
}

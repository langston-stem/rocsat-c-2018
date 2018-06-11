/*
 * Test to run the clinostat motor
 */

// use this to adjust speed. max value is 255. motor won't turn at low values.
const int PWM = 80;

const int pwm_pin = 8;
const int m1_pin = 9;
const int m2_pin = 10;

void setup() {
  pinMode(pwm_pin, OUTPUT);
  pinMode(m1_pin, OUTPUT);
  pinMode(m2_pin, OUTPUT);
}

void loop() {
  analogWrite(pwm_pin, PWM);
  digitalWrite(m1_pin, LOW);
  digitalWrite(m2_pin, HIGH);
}


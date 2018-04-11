
#include<Servo.h>

#define SERVO_MIN 0
#define SERVO_MAX 165

const int led0Pin = 2;
const int led1Pin = 3;
const int led2Pin = 4;
const int led3Pin = 5;
const int led4Pin = 6;
const int led5Pin = 7;
const int led6Pin = 8;
const int led7Pin = 9;
const int servoPin = 10;
const int buttonUpPin = 13;
const int buttonDownPin = 12;

Servo servo;

void setup() {
  // LED pins
  pinMode(led0Pin, OUTPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
  pinMode(led4Pin, OUTPUT);
  pinMode(led5Pin, OUTPUT);
  pinMode(led6Pin, OUTPUT);
  pinMode(led7Pin, OUTPUT);

  // init servo to neutral position
  servo.attach(servoPin);
  servo.write(90);

  // Button pins
  pinMode(buttonUpPin, INPUT);
  pinMode(buttonDownPin, INPUT);
}

int level = 5; // 1 - 9, 5 is neutral
int buttonUpState;
int buttonDownState;

void loop() {
  int state, i;

  state = digitalRead(buttonUpPin);
  if (state && !buttonUpState) {
    level++;
    if (level > 9) {
      level = 9;
    }
  }
  buttonUpState = state;

  state = digitalRead(buttonDownPin);
  if (state && !buttonDownState) {
    level--;
    if (level < 1) {
      level = 1;
    }
  }
  buttonDownState = state;

  for (i = 0; i < 4; i++) {
    state = level <= i + 1 ? HIGH : LOW;
    digitalWrite(led0Pin + i, state);
  }
  for (i = 4; i < 8; i++) {
    state = level > i + 1 ? HIGH : LOW;
    digitalWrite(led0Pin + i, state);
  }
  servo.write(map(level, 1, 9, SERVO_MAX, SERVO_MIN));
}

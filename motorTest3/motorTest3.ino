/*
  Arduino Starter Kit example
  Project 10 - Zoetrope

  This sketch is written to accompany Project 10 in the Arduino Starter Kit

  Parts required:
  - two 10 kilohm resistors
  - two momentary pushbuttons
  - one 10 kilohm potentiometer
  - motor
  - 9V battery
  - H-Bridge

  created 13 Sep 2012
  by Scott Fitzgerald
  Thanks to Federico Vanzati for improvements

  http://www.arduino.cc/starterKit

  This example code is part of the public domain.
*/


#include <Wire.h>
#include "gyro.h"
#include <SPI.h>
#include <SD.h>
#include "SDCard.h"

const int debug = 0;

// Tuning constants
const float Kp = 0.005;
const float Ki = 0.00005;

const int controlPin1 = 7; // connected to pin 7 on the H-bridge
const int controlPin2 = 6; // connected to pin 2 on the H-bridge
const int outBPin = 2; 
const int enablePin = 5;   // connected to pin 1 on the H-bridge
const int directionSwitchPin = 4;  // connected to the switch for direction
const int onOffSwitchStateSwitchPin = 5; // connected to the switch for turning the motor on and off
const int potPin = A0;  // connected to the potentiometer's output
volatile unsigned int motorA_counts = 0;
unsigned int loopCount = 0;
unsigned int debugCount = 0;

// create some variables to hold values from your inputs
int onOffSwitchState = 0;  // current state of the on/off switch
int previousOnOffSwitchState = 0; // previous position of the on/off switch
int directionSwitchState = 0;  // current state of the direction switch
int previousDirectionSwitchState = 0;  // previous state of the direction switch

int motorEnabled = 0; // Turns the motor on/off
int motorPWM = 0; // PWM to motor
int motorSpeed = 0; // actual motor speed from tachometer
int motorDirection = 1; // current direction of the motor

int gyro;
unsigned int startTime;
int errorInt = 0; // integral of error over time

void motorA_int_handler() {
  motorA_counts++;
}

void setup() {
  if (debug) {
    Serial.begin(9600);
  }
  
  Wire.begin();
  SPI.begin ();
  Gyro::setupGyroITG();
  SDCardInit();

  // initialize the inputs and outputs
  pinMode(directionSwitchPin, INPUT);
  pinMode(onOffSwitchStateSwitchPin, INPUT);
  pinMode(outBPin, INPUT);
  pinMode(outBPin, INPUT);
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(outBPin), motorA_int_handler, RISING);

 motorEnabled = 1;

  // pull the enable pin LOW to start
  digitalWrite(enablePin, LOW);
  startTime = millis();
  
    dataString = "gyro, motorSpeed, error, motorPWM";
    writeDataToSD();
}

void loop() {
  // read the value of the on/off switch
//  onOffSwitchState = digitalRead(onOffSwitchStateSwitchPin);
  delay(1);

  // read the value of the direction switch
//  directionSwitchState = digitalRead(directionSwitchPin);

  // read the value of the pot and divide by 4 to get a value that can be
  // used for PWM
  //motorPWM = analogRead(potPin) / 4;
//  gyro = analogRead(potPin) * -3;
   gyro = Gyro::readZ() + 10 /* + INSERT INITIALIZATION VALUE HERE */ ;
 
  // if the on/off button changed state since the last loop()
  if (onOffSwitchState != previousOnOffSwitchState) {
    // change the value of motorEnabled if pressed
    if (onOffSwitchState == HIGH) {
      motorEnabled = !motorEnabled;
    }
  }

  // if the direction button changed state since the last loop()
//  if (directionSwitchState != previousDirectionSwitchState) {
//    // change the value of motorDirection if pressed
//    if (directionSwitchState == HIGH) {
//      motorDirection = !motorDirection;
//    }
//  }

  // change the direction the motor spins by talking to the control pins
  // on the H-Bridge
  if (motorDirection == 1) {
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);
  } else {
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
  }

  // if the motor is supposed to be on
  if (motorEnabled == 1) {
    // PWM the enable pin to vary the speed
    analogWrite(enablePin, motorPWM);
  } else { // if the motor is not supposed to be on
    //turn the motor off
    analogWrite(enablePin, 0);
  }
  // save the current on/off switch state as the previous
  previousDirectionSwitchState = directionSwitchState;
  // save the current switch state as the previous
  previousOnOffSwitchState = onOffSwitchState;

  /****************************************/

  if (loopCount >= 100) {
    unsigned int outACount, now, timeInterval;
    int error;

    now = millis();
    timeInterval = now - startTime;
    startTime = now;

    outACount = motorA_counts;
    // calculate speed in degrees per second.
    // sensor gives 3 counts per rotation, gear ratio is 50:1
    // (motor speed) = (actual counts) / (3 counts/rotation) / (50 gear ratio) * (360 degrees/rotation) / (timeInterval milliseconds)
    motorSpeed = (long)outACount * 2400L / timeInterval;
    motorA_counts = 0;

    // want error to be 0, e.g. motorSpeed is negative gyro speed
    error = gyro - motorSpeed;

    errorInt += error;
    if (errorInt > 5000) {
      errorInt = 5000;
    }
    else if (errorInt < - 5000) {
      errorInt = -5000;
    }

    if (gyro > 0) {
      motorDirection = 1;
    } else {
      motorDirection = 0;
    }

    motorPWM = motorPWM + error * Kp + errorInt * Ki;
    if (motorPWM > 255) {
      motorPWM = 255;
    }
    else if (motorPWM < 0) {
      motorPWM = 0;
    }
    
    loopCount = 0;

    dataString = String(gyro)
      + ", " + String(motorSpeed) 
      + ", " + String(error) 
      + ", " + String(motorPWM);
    writeDataToSD();

    if (debug && debugCount >= 10) {
      Serial.print("gyro: ");
      Serial.println(gyro);
      Serial.print("motorSpeed: ");
      Serial.println(motorSpeed);
      Serial.print("error: ");
      Serial.println(error);
      Serial.print("motorPWM: ");
      Serial.println(motorPWM);
      debugCount = 0;
    } else {
      debugCount++;
    }
  } else {
    loopCount++;
  }
}

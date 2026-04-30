// 45.45 freq
// 1.5ms duration = stop, 2ms duration = full speed forward, 1ms duration = full speed backward
// with 12 bit resolution, this corresponds to duty value 186 to full speed backward, 279 to stop, 372 to full speed forward

#include <Arduino.h>
#include "motors.h"

#define MOTORA_PIN 13    // corresponds to GPIO13 Green
#define MOTORB_PIN 12    // corresponds to GPIO12 Yellow
#define OVERRIDE_PIN 14  // corresponds to GPIO14 Brown
#define KILL_PIN 27      // corresponds to GPIO27 Blue

// pwm constants
const int pwmFreq = 45.45;  // pwm frequency in hz
const int pwmRes = 12;      // 12-bit resolution (0-4095)
const int pwmChannelA = 0;  // pwm channel for MOTORA
const int pwmChannelB = 1;  // pwm channel for MOTORB

// remote control flags
volatile bool overrideFlag = false;
volatile bool killFlag = false;

void overrideISR() {
  overrideFlag = digitalRead(OVERRIDE_PIN) == LOW;
}

void killISR() {
  killFlag = digitalRead(KILL_PIN) == HIGH;
}

void setupMotors() {
  pinMode(OVERRIDE_PIN, INPUT);
  pinMode(KILL_PIN, INPUT);

  ledcAttachChannel(MOTORA_PIN, pwmFreq, pwmRes, pwmChannelA);
  ledcAttachChannel(MOTORB_PIN, pwmFreq, pwmRes, pwmChannelB);

  overrideFlag = digitalRead(OVERRIDE_PIN) == LOW;
  killFlag = digitalRead(KILL_PIN) == HIGH;

  attachInterrupt(digitalPinToInterrupt(OVERRIDE_PIN), overrideISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(KILL_PIN), killISR, CHANGE);
}

// speed input should be -1 to 1, 0 for stop, negatives for reverse direction.
int dutyValue(float speed) {
  speed = constrain(speed, -1, 1);
  int value = 279 + round(speed * 93);  // 279 comes from the duty value at complete motor stop (measured), 93 comes from the difference between the max duty value to achieve full speed (measured), therefore scale accordingly.
  return value;
}

void leftMotor(float speed) {
  int dutyCycle = dutyValue(speed);
  ledcWriteChannel(pwmChannelA, dutyCycle);
}

void rightMotor(float speed) {
  int dutyCycle = dutyValue(speed);
  ledcWriteChannel(pwmChannelB, dutyCycle);
}

bool checkOverride() {
  return overrideFlag;
}

bool checkKill() {
  return killFlag;
}

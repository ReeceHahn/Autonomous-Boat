#ifndef MOTORS_H
#define MOTORS_H

void setupMotors();
void overrideISR();
void killISR();
int dutyValue(float speed);
void leftMotor(float speed);
void rightMotor(float speed);
bool checkOverride();
bool checkKill();

#endif

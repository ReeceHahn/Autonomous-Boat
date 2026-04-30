#ifndef ORIENTATION_H
#define ORIENTATION_H

void setupOrientation();
bool imuAvailable();
void readOrientation();
void readMagnetometer();
void calculateOrientation();
float getIMUHeading();
void printOrientation();

#endif

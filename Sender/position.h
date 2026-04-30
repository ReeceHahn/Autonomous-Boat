#ifndef POSITION_H
#define POSITION_H

void setupPosition();
bool gnssAvailable();
void readPosition();
float getLatitude();
float getLongitude();
float getAltitude();
float getGNSSHeading();
float getSpeed();
int getSIV();
void printPosition();

#endif

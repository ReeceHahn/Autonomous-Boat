#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

void setupKalmanFilter();
void wrapAngleMeasurement(float rawZ, float rawHx, float& zOut, float& hxOut);
void doKalmanFilter();
float getKalmanHeading();
void printKalmanHeading();

#endif

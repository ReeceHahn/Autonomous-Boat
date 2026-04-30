#define EKF_N 2  // states: heading, heading rate
#define EKF_M 1  // measurements: heading

#include <Arduino.h>
#include <tinyekf.h>
#include "position.h"
#include "orientation.h"
#include "kalman_filter.h"

ekf_t ekf;

// state transition matrix
float matrixF[EKF_N * EKF_N] = { 1.0, 0.02, 0.0, 1.0 }; // position 2 is initial dt, assume this to be 0.02 as the loop is 50Hz

// process noise (tunable)
float matrixQ[EKF_N * EKF_N] = { 0.3, 0.0, 0.0, 0.4 };  // position 1 is the uncertainty in the heading reading, position 4 is the uncertainty in the heading change rate, this will be higher than position 1

// measurement function
float matrixH[EKF_M * EKF_N] = { 1.0, 0.0 };

// imu (LSM303D) measurement noise variance
float matrixR_IMU[EKF_M * EKF_M] = { 9.0 };  // ±3° standard deviation

// gnss (NEO-M9N) measurement noise variance
float matrixR_GNSS[EKF_M * EKF_M] = { 2.25 };  // ±1.5° standard deviation

// heading output variable
float kalmanHeading = 0.0;

// timer
unsigned long kalmanLoopTimer = 0;

void setupKalmanFilter() {
  const float Pdiag[EKF_N] = { 1.0, 1.0 };
  ekf_initialize(&ekf, Pdiag);
}

// adjust predicted heading so that the difference from the measured heading is always within ±180, helps the kalman filter not overcompensate when it shouldn't
void wrapAngleMeasurement(float rawZ, float rawHx, float& zOut, float& hxOut) {
  float zWrapped = fmodf(rawZ + 360.0, 360.0);
  float hxWrapped = fmodf(rawHx + 360.0, 360.0);

  if (fabs(zWrapped - hxWrapped) > 180.0) {
    if (zWrapped > hxWrapped) {
      zWrapped -= 360.0;
    } else {
      hxWrapped -= 360.0;
    }
  }

  zOut = zWrapped;
  hxOut = hxWrapped;
}

void doKalmanFilter() {
  if (millis() - kalmanLoopTimer >= 20) {

    float dt = (millis() - kalmanLoopTimer) / 1000.0;  // calculate time between this loop and previous loop in seconds

    matrixF[1] = dt;  // update state transition matrix

    // prediction step
    float fx[EKF_N] = { ekf.x[0] + ekf.x[1] * dt, ekf.x[1] };
    ekf_predict(&ekf, fx, matrixF, matrixQ);

    // imu update (50Hz)
    if (imuAvailable()) {
      float rawZ = getIMUHeading();
      float rawHx = ekf.x[0];
      float z[EKF_M], hx[EKF_M];
      wrapAngleMeasurement(rawZ, rawHx, z[0], hx[0]);
      ekf_update(&ekf, z, hx, matrixH, matrixR_IMU);
    }

    // gnss update (25Hz)
    if ((getSIV() >= 4) && (getSpeed() >= 1.5) && gnssAvailable()) {
      float rawZ = getGNSSHeading();
      float rawHx = ekf.x[0];
      float z[EKF_M], hx[EKF_M];
      wrapAngleMeasurement(rawZ, rawHx, z[0], hx[0]);
      ekf_update(&ekf, z, hx, matrixH, matrixR_GNSS);
    }

    kalmanHeading = fmodf(ekf.x[0] + 720.0, 360.0);  // constrain kalman heading to between 0 and 360

    kalmanLoopTimer = millis();
  }
}

float getKalmanHeading() {
  return kalmanHeading;
}

void printKalmanHeading() {
  Serial.print("EKF Heading: ");
  Serial.print(kalmanHeading);
  Serial.println("°");
}

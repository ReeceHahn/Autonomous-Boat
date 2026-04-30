#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <LSM303.h>  // https://github.com/pololu/lsm303-arduino
#include "orientation.h"

LSM303 LSM303D;

#define GNSS_SDA 18  // corresponds to GPIO18
#define GNSS_SCL 19  // corresponds to GPIO19

// raw imu data scaling values as according to LSM303D datasheet: https://www.pololu.com/file/0J703/LSM303D.pdf
const float magScale = 0.00008;
const float accelScale = 0.000061;

const float declination = 0.41;  // York declination in degrees, source: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm

// magnetometer hard iron bias correction values b (calculated by magneto 1.2 program), THESE WILL NEED TO BE RECALCULATED WHEN MAGNETOMETER IS AT CAMPUS EAST LAKE!
const float magbX = 0.026436;
const float magbY = 0.014348;
const float magbZ = 0.011769;

// magnetometer soft iron bias correcton matrix A^-1 (calculated by magneto 1.2 program), THESE WILL NEED TO BE RECALCULATED WHEN MAGNETOMETER IS AT CAMPUS EAST LAKE!
const float magAinv[3][3] = {
  { 1.005989, -0.007199, 0.001527 },
  { -0.007199, 1.042115, 0.014259 },
  { 0.001527, 0.014259, 1.063146 }
};

// timer
unsigned long imuTimer = 0;

// magnetometer output variables
float magX;
float magY;
float magZ;

// orientation variables
float imuHeading = 0;

void setupOrientation() {
  Wire.begin(GNSS_SDA, GNSS_SCL);
  LSM303D.init();

  // accelerometer initialisation
  LSM303D.writeReg(0x20, 0x57);  // CTRL1 register (0x20) 100Hz output data rate (needs to be over 50Hz if running magnetometer at 50Hz)
  LSM303D.writeReg(0x21, 0x00);  // CTRL2 register (0x21) ±2g resolution

  // magnetometer initialisation
  LSM303D.writeReg(0x24, 0x14);  // CTRL5 register (0x24) 50Hz output data rate
  LSM303D.writeReg(0x25, 0x00);  // CTRL6 register (0x25) ±2 gauss resolution
  LSM303D.writeReg(0x26, 0x00);  // CTRL7 register (0x26) continuous conversion mode
}

bool imuAvailable() {
  if (millis() - imuTimer >= 20) {
    imuTimer = millis();
    return true;
  } else {
    return false;
  }
}

void readOrientation() {
  LSM303D.read();  // read both accelerometer and magnetometer

  readMagnetometer();

  calculateOrientation();
}

void readMagnetometer() {
  // apply scaling to get raw gauss output
  float rawX = LSM303D.m.x * magScale;
  float rawY = LSM303D.m.y * magScale;
  float rawZ = LSM303D.m.z * magScale;

  // correction formula is Ainv(mag - b) from magneto 1.2

  // apply hard iron correction
  float hardX = rawX - magbX;
  float hardY = rawY - magbY;
  float hardZ = rawZ - magbZ;

  // apply soft iron correction
  float softX = magAinv[0][0] * hardX + magAinv[0][1] * hardY + magAinv[0][2] * hardZ;
  float softY = magAinv[1][0] * hardX + magAinv[1][1] * hardY + magAinv[1][2] * hardZ;
  float softZ = magAinv[2][0] * hardX + magAinv[2][1] * hardY + magAinv[2][2] * hardZ;

  magX = softX;
  magY = softY;
  magZ = softZ;

  /*
  // print raw values for calibration purposes
  Serial.print(rawX, 5);
  Serial.print(",");
  Serial.print(rawY, 5);
  Serial.print(",");
  Serial.print(rawZ, 5);
  Serial.println("");
  */
}

void calculateOrientation() {
  float headingRad = atan2(magY, magX);                    // calculate heading in radians, will calculate heading in x direction shown on lsm303d breakout board
  float headingDeg = degrees((headingRad) + declination);  // convert value to degrees, factor in magnetic declination

  imuHeading = fmodf(headingDeg + 360.0, 360.0);
}

float getIMUHeading() {
  return imuHeading;
}

void printOrientation() {
  Serial.print("IMU Heading: ");
  Serial.print(imuHeading, 2);
  Serial.println("°");
}

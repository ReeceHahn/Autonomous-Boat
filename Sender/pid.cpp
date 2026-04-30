#include <Arduino.h>
#include <math.h>
#include "motors.h"
#include "position.h"
#include "orientation.h"
#include "kalman_filter.h"
#include "pid.h"

// pid constants
const float kp = 0.005;
const float ki = 0.00001;
const float kd = 0.00005;

const float maxIntegralCorrection = 0.15;
const float maxIntegral = maxIntegralCorrection / ki;
const float maxSpeed = 0.65;

unsigned long pidTimer = 0;

float prevError = 0;
float integral = 0;

float calcTargetHeading(float waypointLat, float waypointLon) {
  float lat = getLatitude();
  float lon = getLongitude();

  // latitude and longitude values are given in degrees, so need to convert them to radians
  lat = radians(lat);
  lon = radians(lon);
  waypointLat = radians(waypointLat);
  waypointLon = radians(waypointLon);

  // initial bearing formula for heading from two points across a sphere
  float y = sin(waypointLon - lon) * cos(waypointLat);
  float x = cos(lat) * sin(waypointLat) - sin(lat) * cos(waypointLat) * cos(waypointLon - lon);
  float headingRad = atan2(y, x);
  float headingDeg = degrees(headingRad);

  return fmodf((headingDeg + 360.0), 360.0);  // normalise result to be between 0 and 360 degrees
}

float calcHeadingError(float currentHeading, float targetHeading) {
  float error = targetHeading - currentHeading;

  // normalise the error between -180 and 180 degrees
  if (error > 180) error -= 360.0;
  if (error < -180) error += 360.0;

  return error;
}

// need to call this when updating new waypoint value! important
void resetPID() {
  prevError = 0;
  integral = 0;
}

void pidControl(float waypointLat, float waypointLon) {
  if (millis() - pidTimer >= 20) {
    float leftMotorSpeed, rightMotorSpeed;

    float targetHeading = calcTargetHeading(waypointLat, waypointLon);  // calculate target heading

    float error = calcHeadingError(getKalmanHeading(), targetHeading);  // calculate error between current heading and target heading normalised between -180 and 180

    float dt = (millis() - pidTimer) / 1000.0;  // dt in seconds

    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral);  // keep integral value from drifting too far and causing destablisation
    float derivative = (error - prevError) / dt;
    prevError = error;

    float correction = (kp * error) + (ki * integral) + (kd * derivative);

    correction = constrain(correction, -1.0, 1.0);  // constrain correction between -1 and 1

    if (correction > 0) {
      leftMotorSpeed = (1.0 - correction) * maxSpeed;  // if the correction is positive, that means we must turn right therefore need to slow the left motor
      rightMotorSpeed = maxSpeed;
    } else {
      rightMotorSpeed = (1.0 + correction) * maxSpeed;  // if the correction is negative, that means we must turn left therefore need to slow the right motor
      leftMotorSpeed = maxSpeed;
    }

    leftMotorSpeed = constrain(leftMotorSpeed, 0.0, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0.0, maxSpeed);

    leftMotor(leftMotorSpeed);
    rightMotor(rightMotorSpeed);

    pidTimer = millis();
  }
}

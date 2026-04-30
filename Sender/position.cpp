#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
#include "position.h"

SFE_UBLOX_GNSS GNSS;

#define GNSS_SDA 18  // corresponds to GPIO18
#define GNSS_SCL 19  // corresponds to GPIO19

// timer
unsigned long gnssTimer = 0;

// position variables
float gnssLatitude = 0.0;
float gnssLongitude = 0.0;
float gnssAltitude = 0.0;
float gnssHeading = 0.0;
float gnssSpeed = 0.0;
int gnssSIV = 0;

void setupPosition() {
  Wire.begin(GNSS_SDA, GNSS_SCL);
  GNSS.begin(Wire);

  GNSS.setI2COutput(COM_TYPE_UBX);      // turns off NMEA noise
  GNSS.setDynamicModel(DYN_MODEL_SEA);  // set dynamic model as a boat
  GNSS.setNavigationFrequency(25);      // max is 25hz
  GNSS.setAutoPVT(true);                // ensure position and heading updates happen automatically
  GNSS.saveConfiguration();             // save this new frequency
}

bool gnssAvailable() {
  if (millis() - gnssTimer >= 40) {
    gnssTimer = millis();
    return true;
  } else {
    return false;
  }
}

void readPosition() {
  if (GNSS.getPVT()) {
    if (GNSS.getGnssFixOk() && GNSS.getFixType() >= 3) {
      gnssLatitude = GNSS.getLatitude() / 1.0e7;
      gnssLongitude = GNSS.getLongitude() / 1.0e7;
      gnssAltitude = GNSS.getAltitudeMSL() / 1.0e3;
      gnssHeading = GNSS.getHeading() / 1.0e5;
      gnssSpeed = GNSS.getGroundSpeed() / 1.0e3;
    }

    gnssSIV = GNSS.getSIV();
  }
}

float getLatitude() {
  return gnssLatitude;
}

float getLongitude() {
  return gnssLongitude;
}

float getAltitude() {
  return gnssAltitude;
}

float getGNSSHeading() {
  return gnssHeading;
}

float getSpeed() {
  return gnssSpeed;
}

int getSIV() {
  return gnssSIV;
}

void printPosition() {
  Serial.print("Lat: ");
  Serial.print(gnssLatitude, 7);
  Serial.print("°");
  Serial.print("\tLon: ");
  Serial.print(gnssLongitude, 7);
  Serial.print("°");
  Serial.print("\tAlt: ");
  Serial.print(gnssAltitude, 2);
  Serial.print("m");
  Serial.print("\tHeading: ");
  Serial.print(gnssHeading, 2);
  Serial.print("°");
  Serial.print("\tSpeed: ");
  Serial.print(gnssSpeed, 2);
  Serial.print("m/s");
  Serial.print("\tSIV: ");
  Serial.println(gnssSIV);
}

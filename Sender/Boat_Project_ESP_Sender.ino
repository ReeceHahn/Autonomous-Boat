#include <math.h>
#include "lora.h"
#include "motors.h"
#include "position.h"
#include "orientation.h"
#include "kalman_filter.h"
#include "pid.h"
#include "sensors.h"
#include "pump.h"

enum boatStates {
  TESTING,     // boat is being tested on with temporary functions
  IDLE,        // boat is idle waiting for starting processes to finish
  NAVIGATING,  // autonomously navigating to a waypoint
  SAMPLING,    // at a waypoint, checking water quality
  COLLECTING,  // at a waypoint, detects atypical reading, activates water pump to collect sample
  RETURNING,   // returning to shore
  MANUAL,      // user is actively overriding the boat
  COMPLETE     // mission is finished
};

boatStates boatStatus = IDLE;          // boat starts off as idle
boatStates previousBoatStatus = IDLE;  // variable to track previous boat state for debugging purposes

float waypoints[6][2] = {
  { 53.946100, -1.027600 },  // waypoint 1
  { 53.946190, -1.026640 },  // waypoint 2
  { 53.946470, -1.026640 },  // waypoint 3
  { 53.946430, -1.027750 },  // waypoint 4
  { 53.946500, -1.026200 },  // waypoint 5
  { 53.946610, -1.026970 }   // starting position, will be updated in setup
};

const int shoreIndex = 5;

int waypointIndex = 0;

const float waypointError = 8.0;  // the radius in meters around a waypoint in which it is acceptable for the boat to stop

// atypical reading limit values
const float temperatureUpperLimit = 20.0;
const float temperatureLowerLimit = 10.0;
const float phUpperLimit = 9.5;
const float phLowerLimit = 5.5;
const float turbidityUpperLimit = 25;

const float earthRadius = 6378137.0;  // in meters

// durations
const unsigned long printDuration = 500;    // ms -> 0.5 seconds
const unsigned long sampleDuration = 8000;  // ms -> 8 seconds
const unsigned long pumpDuration = 2500;    // ms -> 2.5 seconds

// timers
unsigned long printTimer = 0;
unsigned long sampleTimer = 0;
unsigned long pumpTimer = 0;
unsigned long tempTimer = 0;

// sampling
const float sampleRate = 1.25;  // hz
const int sampleCount = sampleRate * (sampleDuration / 1000);
int sampleIndex = 0;

float temperatureSamples[sampleCount];
float phSamples[sampleCount];
float turbiditySamples[sampleCount];

// flags
bool serialFlag = true;

float haversineDistance(float lat1, float lon1, float lat2, float lon2) {
  float deltaLat = radians(lat2 - lat1);
  float deltaLon = radians(lon2 - lon1);

  float a = sq(sin(deltaLat / 2)) + cos(radians(lat1)) * cos(radians(lat2)) * sq(sin(deltaLon / 2));

  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return earthRadius * c;
}

float calculateMean(float* data, int n) {
  float sum = 0;

  for (int i = 0; i < n; ++i) {
    sum += data[i];
  }

  return sum / n;
}

void stopBoat() {
  leftMotor(0);
  rightMotor(0);
}

void startBoat() {
  // remain idle while gnss is not connected to atleast 4 satellites
  if (getSIV() >= 4) {

    // set the boat's starting position to its current position
    waypoints[shoreIndex][0] = getLatitude();
    waypoints[shoreIndex][1] = getLongitude();

    boatStatus = NAVIGATING;
  }
}

void startSampling() {
  sampleTimer = millis();
  sampleIndex = 0;
}

void startCollecting() {
  pumpTimer = millis();
}

void sampleWater() {
  if (sensorsAvailable()) {
    if (sampleIndex < sampleCount) {
      temperatureSamples[sampleIndex] = getTemperature();
      phSamples[sampleIndex] = getPH();
      turbiditySamples[sampleIndex] = getTurbidity();
      sampleIndex++;
    } else {
      checkWater();
    }
  }
}

void checkWater() {
  float temperature = calculateMean(temperatureSamples, sampleCount);
  float ph = calculateMean(phSamples, sampleCount);
  float turbidity = calculateMean(turbiditySamples, sampleCount);

  if (temperature <= temperatureLowerLimit || temperature >= temperatureUpperLimit || ph <= phLowerLimit || ph >= phUpperLimit || turbidity >= turbidityUpperLimit) {
    boatStatus = COLLECTING;
    startCollecting();
  } else {
    resetPID();
    if (waypointIndex == shoreIndex) {
      boatStatus = RETURNING;
    } else {
      boatStatus = NAVIGATING;
    }
  }
}

void collectWater() {
  if (millis() - pumpTimer <= pumpDuration) {
    activatePump();
  } else {
    deactivatePump();
    waypointIndex = shoreIndex;
    boatStatus = RETURNING;
  }
}

void checkDestination() {
  float latitude = getLatitude();
  float longitude = getLongitude();

  float targetLat = waypoints[waypointIndex][0];
  float targetLon = waypoints[waypointIndex][1];

  float distance = haversineDistance(latitude, longitude, targetLat, targetLon);

  if (distance <= waypointError) {

    stopBoat();
    resetPID();

    if (boatStatus == RETURNING) {
      boatStatus = COMPLETE;
      return;
    }

    if (waypointIndex < shoreIndex) {
      waypointIndex++;
    }

    boatStatus = SAMPLING;
    startSampling();
  }
}

bool checkAutonomous() {
  if (!checkOverride() && !checkKill()) {
    return true;
  } else {
    return false;
  }
}

String getBoatStatus() {
  switch (boatStatus) {
    case TESTING: return "TESTING";
    case IDLE: return "IDLE";
    case NAVIGATING: return "NAVIGATING";
    case SAMPLING: return "SAMPLING";
    case COLLECTING: return "COLLECTING";
    case RETURNING: return "RETURNING";
    case MANUAL: return "MANUAL";
    case COMPLETE: return "COMPLETE";
    default: return "UNKNOWN";
  }
}

void printStatusChange() {
  if (serialFlag) {
    if (boatStatus != previousBoatStatus) {
      Serial.println("Status changed from " + String(previousBoatStatus) + " to " + String(boatStatus));
      previousBoatStatus = boatStatus;
    }
  }
}

void printValues() {
  if (serialFlag) {
    if (millis() - printTimer >= printDuration) {
      printSensors();
      printPosition();
      printOrientation();
      printKalmanHeading();
      Serial.println("");

      printTimer = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);

  setupLora();
  setupOrientation();
  setupPosition();
  setupKalmanFilter();
  setupMotors();
  setupSensors();
  setupPump();
}

void loop() {
  printStatusChange();  // check if the boat's status has changed and if so, notify serial and bluetooth of this change

  // check the boat's autonomy status
  if (checkAutonomous() == false) {
    if (boatStatus != MANUAL) {
      stopBoat();
      resetPID();
      boatStatus = MANUAL;
    }
  } else if (boatStatus == MANUAL) {
    boatStatus = NAVIGATING;
    resetPID();
  }

  // check the boat's state and perform actions as required
  switch (boatStatus) {
    // logic for testing phases
    case TESTING:
      // any functionality you want to test goes here!!!!!!!!!!!!!!!!! <---------------------------------------------------------------
      receiveLora();      // receive any lora waypoint updates
      readOrientation();  // read the boat's current heading
      readPosition();     // read the boat's current coordinates
      doKalmanFilter();   // perform kalman filter on imu heading and gnss heading
      readSensors();      // read the boat's water property sensors
      printValues();      // print all reading values to serial
      sendLora();         // send lora communication back to base station
      break;

    // logic for the boat when idle
    case IDLE:
      receiveLora();      // receive any lora waypoint updates
      readOrientation();  // read the boat's current heading
      readPosition();     // read the boat's current coordinates
      doKalmanFilter();   // perform kalman filter on imu heading and gnss heading
      readSensors();      // attempt to read the boat's water sensors
      startBoat();        // attempt to start boat by checking how many gnss satellites are connected
      sendLora();         // send lora communication back to base station
      break;

    // logic for the boat when autonomously navigating
    case NAVIGATING:
      receiveLora();                                                         // receive any lora waypoint updates
      readOrientation();                                                     // read the boat's current heading
      readPosition();                                                        // read the boat's current coordinates
      doKalmanFilter();                                                      // perform kalman filter on imu heading and gnss heading
      readSensors();                                                         // read the boat's water property sensors
      pidControl(waypoints[waypointIndex][0], waypoints[waypointIndex][1]);  // perform pid speed control on boat's motors to navigate to waypoint
      checkDestination();                                                    // check to see if boat's location matches that of the allowed radius around the specified waypoints
      sendLora();                                                            // send lora communication back to base station
      break;

    // logic for the boat when sampling the waypoints
    case SAMPLING:
      receiveLora();      // receive any lora waypoint updates
      readOrientation();  // read the boat's current heading
      readPosition();     // read the boat's current coordinates
      doKalmanFilter();   // perform kalman filter on imu heading and gnss heading
      readSensors();      // read the boat's water property sensors
      sampleWater();      // sample the water's properties a sufficient amount of times, compare mean of results to atypical value limits
      sendLora();         // send lora communication back to base station
      break;

    // logic for the boat when actively collecting water sample
    case COLLECTING:
      collectWater();     // collect sufficient volume of water from waypoint
      receiveLora();      // receive any lora waypoint updates
      readOrientation();  // read the boat's current heading
      readPosition();     // read the boat's current coordinates
      doKalmanFilter();   // perform kalman filter on imu heading and gnss heading
      readSensors();      // read the boat's water property sensors
      sendLora();         // send lora communication back to base station
      break;

    // logic for the boat when returning back to shore
    case RETURNING:
      receiveLora();                                                   // receive any lora waypoint updates
      readOrientation();                                               // read the boat's current heading
      readPosition();                                                  // read the boat's current coordinates
      doKalmanFilter();                                                // perform kalman filter on imu heading and gnss heading
      readSensors();                                                   // read the boat's water property sensors
      pidControl(waypoints[shoreIndex][0], waypoints[shoreIndex][1]);  // perform pid speed control on boat's motors to navigate to shore
      checkDestination();                                              // check to see if boat's location matches that of the allowed radius around the specified waypoints
      sendLora();                                                      // send lora communication back to base station
      break;

    // logic for the boat when under manual operation mode
    case MANUAL:
      receiveLora();       // receive any lora waypoint updates
      readOrientation();   // read the boat's current heading
      readPosition();      // read the boat's current coordinates
      doKalmanFilter();    // perform kalman filter on imu heading and gnss heading
      readSensors();       // read the boat's water property sensors
      checkDestination();  // check to see if boat's location matches that of the allowed radius around the specified waypoints
      stopBoat();          // ensure boat is set to 0% speed in the code
      sendLora();          // send lora communication back to base station
      break;

    // logic for the boat when the mission is completed
    case COMPLETE:
      receiveLora();      // receive any lora waypoint updates
      readOrientation();  // read the boat's current heading
      readPosition();     // read the boat's current coordinates
      doKalmanFilter();   // perform kalman filter on imu heading and gnss heading
      readSensors();      // read the boat's water property sensors
      stopBoat();         // ensure boat is stationary and awaiting pickup
      sendLora();         // send lora communication back to base station
      break;
  }
}

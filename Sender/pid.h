#ifndef PID_H
#define PID_H

float calcTargetHeading(float waypointLat, float waypointLon);
float calcHeadingError(float currentHeading, float targetHeading);
void resetPID();
void pidControl(float waypointLat, float waypointLon);

#endif

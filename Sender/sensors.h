#ifndef SENSORS_H
#define SENSORS_H

void setupSensors();
bool sensorsAvailable();
void readSensors();
void readTemperature();
void readPH();
void readTurbidity();
float getTemperature();
float getPH();
float getTurbidity();
void printSensors();

#endif

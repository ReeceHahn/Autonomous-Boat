#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>  // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "sensors.h"

#define TEMPERATURE_PIN 4  // corresponds to GPIO4
#define PH_PIN 15          // corresponds to GPIO15
#define TURBIDITY_PIN 34   // corresponds to GPIO34

const float espADC = 4095.0;   // the esp Analog Digital Convertion value
const float espVoltage = 3.3;  // the esp voltage supply value

OneWire oneWire(TEMPERATURE_PIN);  // setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)

DallasTemperature sensors(&oneWire);  // pass our oneWire reference to Dallas Temperature

// constants
const unsigned long temperatureConversionDelay = 800;  // for 9 bit resolution there is about 94ms conversion delay, that means we cant call tempC within this time or else the program will be blocked until the 94ms have passed

// timer
unsigned long sensorsTimer = 0;
unsigned long temperatureTimer = 0;

// sensor variables set to default values
float temperature = 20;
float ph = 7;
float turbidity = 0;

void setupSensors() {
  EEPROM.begin(32);  // needed to permit storage of calibration value in eeprom

  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(TURBIDITY_PIN, INPUT);

  sensors.setWaitForConversion(false);  // tells library not to block
  sensors.begin();
}

bool sensorsAvailable() {
  if (millis() - sensorsTimer >= 800) {
    sensorsTimer = millis();
    return true;
  } else {
    return false;
  }
}

void readSensors() {
  readTemperature();
  readPH();
  readTurbidity();
}

void readTemperature() {
  static enum { IDLE,
                WAITING } state = IDLE;

  switch (state) {
    case IDLE:
      sensors.requestTemperatures();  // starts conversion
      temperatureTimer = millis();    // start timer
      state = WAITING;
      break;

    case WAITING:
      if (millis() - temperatureTimer >= temperatureConversionDelay) {
        float tempC = sensors.getTempCByIndex(0);  // read when ready
        if (tempC != DEVICE_DISCONNECTED_C) {
          temperature = tempC - 1.0;
        }
        state = IDLE;
      }
      break;
  }
}

void readPH() {
  float rawADC = analogRead(PH_PIN);
  float voltage = (rawADC / espADC) * espVoltage * 1000.0;  // read the voltage in millivolts

  ph = 4.0 + (3.0 * (voltage - 915.0) / 230.0); // linear interpolation for mV to ph scaling
}

void readTurbidity() {
  float rawADC = analogRead(TURBIDITY_PIN);
  float rawVoltage = (rawADC / espADC) * espVoltage * 2.0;  // read the raw voltage input, scale it up due to voltage divider
  float voltage = 0.997 * rawVoltage + 0.242;               // correct this raw voltage to what is actually being output by turbidity sensor, values gathered by testing input voltages and adc output voltages, applied linear regression

  turbidity = 6003.7924 * exp(-1.8062 * voltage) + 1.2278;  // convert voltage to turbidity in NTU, formula calculated by assuming exponential regression on values of voltage tested against known turbidities by other people

  if (turbidity > 3000) {
    turbidity = 3000;  // 3000 is the max reading of the sensor so any reading above this value is useless
  }
  if (turbidity < 0) {
    turbidity = 0;  // negative turbidity does not exist
  }
}

float getTemperature() {
  return temperature;
}

float getPH() {
  return ph;
}

float getTurbidity() {
  return turbidity;
}

void printSensors() {
  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.print("°C");
  Serial.print("\tpH: ");
  Serial.print(ph, 2);
  Serial.print("\tTurbidity: ");
  Serial.print(turbidity, 2);
  Serial.println(" NTU");
}

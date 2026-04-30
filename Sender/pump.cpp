#include <Arduino.h>
#include "pump.h"

#define PUMP_PIN 32  // corresponds to GPIO32

void setupPump() {
  pinMode(PUMP_PIN, OUTPUT);

  deactivatePump();  // make sure pump is turned off initially
}

void activatePump() {
  digitalWrite(PUMP_PIN, HIGH);
}

void deactivatePump() {
  digitalWrite(PUMP_PIN, LOW);
}

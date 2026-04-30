#include <Arduino.h>
#include "sensors.h"
#include "position.h"
#include "orientation.h"
#include "kalman_filter.h"
#include "lora.h"
#include "pid.h"

extern float waypoints[6][2];
extern int waypointIndex;
extern String getBoatStatus();

// UART connections
#define rx2 16
#define tx2 17

unsigned long loraTimer = 0;

const char startChar = 'T';
const char endChar = '$';

void setupLora() {
  Serial2.begin(9600, SERIAL_8N1, rx2, tx2);
}

// lat, lon, alt, siv, speed, turbidity, status, heading, temperature, ph, waypoint index
String getCombinedString() {
  String combinedString = String(startChar) + String(getLatitude(), 7) + ":" + String(getLongitude(), 7) + ":" + String(getAltitude(), 2) + ":" + String(getSIV()) + ":" + String(getSpeed(), 2) + ":" + String(getTurbidity(), 2) + ":" + String(getBoatStatus()) + ":" + String(getKalmanHeading(), 2) + ":" + String(getTemperature(), 2) + ":" + String(getPH(), 2) + ":" + String(waypointIndex) + String(endChar);
  return combinedString;
}

void printWaypoints() {
  String message = "Waypoints:\n";
  for (int i = 0; i < 6; i++) {  // 6 waypoints
    message += "W" + String(i + 1) + ": " + String(waypoints[i][0], 6) + ", " + String(waypoints[i][1], 6) + "\n";
  }
  Serial.println(message);
}

void sendLora() {
  if (millis() - loraTimer >= 1000) {
    Serial2.print(getCombinedString());  // send message in one go
    loraTimer = millis();
  }
}

void receiveLora() {
  if (Serial2.available()) {
    String message = Serial2.readStringUntil('$');
    message.trim();

    if (message.startsWith("W:")) {
      // store old waypoints in a temp array to be compared to updated waypoints
      float oldWaypoints[6][2];
      for (int i = 0; i < 6; i++) {
        oldWaypoints[i][0] = waypoints[i][0];
        oldWaypoints[i][1] = waypoints[i][1];
      }

      String coordinateString = message.substring(2);  // strip "W:"
      const int valuesTotal = 10;
      int valuesCount = 0;
      int startIndex = 0;

      while (valuesCount < valuesTotal) {
        int nextIndex = coordinateString.indexOf(':', startIndex);

        String part = (nextIndex == -1) ? coordinateString.substring(startIndex)
                                        : coordinateString.substring(startIndex, nextIndex);

        float value = part.toFloat();
        int row = valuesCount / 2;
        int col = valuesCount % 2;

        // detect and handle incomplete message
        if (nextIndex == -1 && valuesCount < 9) {
          return;
        }

        // only update waypoints if message is not incomplete
        waypoints[row][col] = value;

        startIndex = nextIndex + 1;
        valuesCount++;
      }

      // find the index of the first changed waypoint
      int firstChangedIndex = -1;
      for (int i = 0; i < 6; i++) {
        if (waypoints[i][0] != oldWaypoints[i][0] || waypoints[i][1] != oldWaypoints[i][1]) {
          firstChangedIndex = i;
          break;
        }
      }

      // set the waypoint index to the first changed waypoint so that the boat does not have to repeat the same waypoints if they did not change
      if (firstChangedIndex != -1 && firstChangedIndex <= waypointIndex) {
        waypointIndex = firstChangedIndex;
        resetPID();
      }
    }

    printWaypoints();
  }
}

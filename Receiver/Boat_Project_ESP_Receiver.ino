#include <HardwareSerial.h>

#include <BluetoothSerial.h>  //ESP32 bluetooth library https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial/examples

//Creation of the bluetooth object
BluetoothSerial SerialBT;

// LoRa Serial Pins
#define RXp2 16
#define TXp2 17
HardwareSerial LoRaSerial(2);

// Message markers
char StartChar = 'T';
char EndChar = '$';
char BreakChar = ':';

//Control for the bluetooth or serial enabled
int BluetoothEnabled = 1;
int SerialEnabled = 0;

// Data arrays
const int DataN = 10;
String Data[DataN];
float Latitude, Longitude, Altitude, SIV;
float Speed, Turbidity, Heading;
String Status;
float Temperature, pH;

// Waypoints: 5 pairs of lat/lon
float Waypoints[5][2];

// Setup
void setup() {
  Serial.begin(9600);
  SerialBT.begin("Base Station");  //Bluetooth device name
  LoRaSerial.begin(9600, SERIAL_8N1, RXp2, TXp2);
  Serial.println("ESP32 LoRa Receiver Ready");
}

// Main loop
void loop() {
  if (LoRaSerial.available()) {
    String msg = LoRaSerial.readStringUntil(EndChar);
    msg.trim();
    Serial.println("Received from LoRa =" + msg);

    if (msg.startsWith("W:")) {
      updateWaypoints(msg.substring(2));
      PrintMessageLn("Waypoints updated from LoRa: " + msg.substring(2));
    } else {
      parseSensorData(msg);
    }

    while (LoRaSerial.available()) LoRaSerial.read();
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      // Forward to LoRa
      LoRaSerial.print(input);
      LoRaSerial.print(EndChar);  // End character for LoRa protocol
      //Serial.println("Forwarded to LoRa: " + input);

      // Forward to Bluetooth
      PrintMessageLn("From PC: " + input);

      // Optionally update local waypoints if it's a waypoint message
      if (input.startsWith("W:")) {
        updateWaypoints(input.substring(2));
      }
    }
  }

  delay(100);
}


// Parse GPS/IMU sensor message
void parseSensorData(String message) {
  int Indexes[DataN + 1];

  // Find all separator positions
  for (int i = 0; i <= DataN; i++) {
    Indexes[i] = (i == 0) ? 0 : message.indexOf(BreakChar, Indexes[i - 1] + 1);
  }

  // Extract each field
  for (int j = 0; j < DataN; j++) {
    Data[j] = (j == 0) ? message.substring(0, Indexes[1])
                       : message.substring(Indexes[j] + 1, Indexes[j + 1]);
  }

  // Store into variables
  Latitude = Data[0].toFloat();
  Longitude = Data[1].toFloat();
  Altitude = Data[2].toFloat();
  SIV = Data[3].toFloat();
  Speed = Data[4].toFloat();
  Turbidity = Data[5].toFloat();
  Status = Data[6].toFloat();
  Heading = Data[7].toFloat();
  Temperature = Data[8].toFloat();
  pH = Data[9].toFloat();

  // Print in CSV
  Serial.print("DATA: ");
  Serial.print(Latitude, 6);
  Serial.print(",");
  Serial.print(Longitude, 6);
  Serial.print(",");
  Serial.print(Altitude, 2);
  Serial.print(",");
  Serial.print(SIV);
  Serial.print(",");
  Serial.print(Speed);
  Serial.print(",");
  Serial.print(Turbidity);
  Serial.print(",");
  Serial.print(Status);
  Serial.print(",");
  Serial.print(Heading);
  Serial.print(",");
  Serial.print(Temperature);
  Serial.print(",");
  Serial.println(pH);
}

// Parse and store new waypoint list
void updateWaypoints(String payload) {
  Serial.println("Updating waypoints...");

  int count = 0;
  int startIdx = 0;
  while (count < 10) {
    int nextIdx = payload.indexOf(BreakChar, startIdx);
    if (nextIdx == -1 && count < 9) {
      Serial.println("Incomplete waypoint data");
      return;
    }

    String part = (nextIdx == -1) ? payload.substring(startIdx)
                                  : payload.substring(startIdx, nextIdx);

    float val = part.toFloat();
    int pair = count / 2;
    int axis = count % 2;
    Waypoints[pair][axis] = val;

    startIdx = nextIdx + 1;
    count++;
  }

  Serial.println("Waypoints updated:");
  for (int i = 0; i < 5; i++) {
    Serial.printf("  W%d: %.6f, %.6f\n", i + 1, Waypoints[i][0], Waypoints[i][1]);
  }
}

void PrintMessage(String Message) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
  if (SerialEnabled == 1) {
    Serial.print(Message);
  }
  if (BluetoothEnabled == 1) {  //Doesnt print bluetooth message, only adds to buffer. Will be printed once the PrintMessageLn() command is used. Is due to library problems
    uint8_t buf[Message.length()];
    memcpy(buf, Message.c_str(), Message.length());
    SerialBT.write(buf, Message.length());
  }
}

void PrintMessageLn(String MessageLn) {  //Used to send outputs over both serial and IR, used to make other sections cleaner
  if (SerialEnabled == 1) {
    Serial.println(MessageLn);
  }
  if (BluetoothEnabled == 1) {  //Assembles the message, writes it to buffer then sends buffer
    uint8_t buf[MessageLn.length()];
    memcpy(buf, MessageLn.c_str(), MessageLn.length());
    SerialBT.write(buf, MessageLn.length());
    SerialBT.println();
  }
}

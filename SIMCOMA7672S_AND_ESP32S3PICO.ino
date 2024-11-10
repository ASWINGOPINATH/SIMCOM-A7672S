#include <HardwareSerial.h>

// Define UART for SIMCOM A7672S
HardwareSerial simcom(1);  // Use UART1 on the ESP32-S3 Pico
const int RXD_PIN = 16;     // ESP32 RX pin connected to SIMCOM TX
const int TXD_PIN = 17;     // ESP32 TX pin connected to SIMCOM RX
const int EN_GPIO = 32;     // GPIO to control SIMCOM enable (if required)

// Flags and variables
bool mqttConnectFlag = false;
bool nextCommand = true;
bool disconnectMqtt = false;
bool correctTimeFromSntp = false;

char longitudeString[50];
char latitudeString[50];

void setup() {
  Serial.begin(115200);      // Start Serial Monitor
  simcom.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);  // Initialize SIMCOM UART
  pinMode(EN_GPIO, OUTPUT);  // Set GPIO as output (if using EN pin)
  digitalWrite(EN_GPIO, HIGH);  // Enable SIMCOM module (if using EN pin)

  delay(1000);
  sendAtCommand("AT+CPIN?\r\n");  // Send initial AT command to check SIM status
}

void loop() {
  if (simcom.available()) {
    String response = simcom.readString();
    Serial.println(response);  // Print the response to Serial Monitor
    parseSimcomResponse(response);
  }
}

void sendAtCommand(const char* command) {
  Serial.print("Sending command: ");
  Serial.println(command);
  simcom.print(command);  // Send AT command to SIMCOM module
}

void parseSimcomResponse(const String& data) {
  if (data.indexOf("OK") >= 0) {
    Serial.println("AT Command successful");
  } else if (data.indexOf("ERROR") >= 0) {
    Serial.println("AT Command failed");
    nextCommand = false;
    disconnectMqtt = true;
  } else if (data.indexOf("+CSQ:") >= 0) {
    int startIdx = data.indexOf("+CSQ:") + 6;
    int signalStrength = data.substring(startIdx, startIdx + 2).toInt();
    Serial.print("Signal Strength: ");
    Serial.println(signalStrength);
  } else if (data.indexOf("+CLBS:") >= 0) {
    int startIdx = data.indexOf(",") + 1;
    int endIdx = data.indexOf(",", startIdx);
    String latitude = data.substring(startIdx, endIdx);
    startIdx = endIdx + 1;
    endIdx = data.indexOf(",", startIdx);
    String longitude = data.substring(startIdx, endIdx);
    
    latitude.toCharArray(latitudeString, 50);
    longitude.toCharArray(longitudeString, 50);
    
    Serial.print("Latitude: ");
    Serial.println(latitudeString);
    Serial.print("Longitude: ");
    Serial.println(longitudeString);
  } else if (data.indexOf("CMQTTCONNECT") >= 0) {
    if (data.indexOf("0,0") >= 0) {
      Serial.println("MQTT Connected");
      mqttConnectFlag = true;
    }
  } else if (data.indexOf("+CNTP:") >= 0) {
    if (data.indexOf("0") >= 0) {
      correctTimeFromSntp = true;
    } else {
      correctTimeFromSntp = false;
    }
  }
}

/*
  Basic control of a given wireless cart from a separate arduino
  Only reads out sensor information
*/

#include <ArduinoBLE.h>
#include "Joystick.h"

// UUIDs defined by cart script
#define CART_UUID "bac3"
#define DATA_UUID "78d3"
//#define COMMAND_UUID "2bef"

// Global variables for BLE tracking
BLEDevice dev;
bool scanning = false;

BLECharacteristic sensorData;
//BLECharacteristic wheelRef;

long lastMillis = millis();

// Joystick Objects from Custom Class
Joystick jsA(A0);
Joystick jsB(A1);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Joystick Calibration");
  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());


  Serial.println("Initializing BLE Service");
  if(!BLE.begin()) {
    Serial.println("Failed to Initialize BLE");
    while(1);
  }
  BLE.setLocalName("ArduinoJoystickCentral");

  // Initialize connection off
  disconnectionLED();
}

void loop() {
  // Check for connection/scan if need be
  if(!connectedScan()) return;

  // Normal Operation
  if(sensorData.valueUpdated()) {
    float data[9];
    sensorData.readValue(&data, 36);
    Serial.println(String(data[0]) + ' ' + String(data[1]) + ' ' + String(data[2]) + String(data[8]));
  }
  int wheelSpeeds[2];
  JoystickControl(wheelSpeeds, 255);
  //Serial.println(String(wheelSpeeds[0]) + ' ' + String(wheelSpeeds[1]));
  //sensorData.writeValue(&wheelSpeeds, 8);
  
  
    
}

long logTime() {
  long temp = millis() - lastMillis;
  lastMillis = millis();
  return temp;
}

// reads joystick inputs, converts to wL and wR
void JoystickControl(int* w, int maxSpeed) {
  float Ain = jsA.read();
  float Bin = jsB.read();

  // omega factor: -1 to 1
  float wf_L = Ain + Bin;  
  float wf_R = Ain - Bin;
  if(abs(wf_L)>1)
    wf_L /= abs(wf_L);
  if(abs(wf_R)>1)
    wf_R /= abs(wf_R);

  w[0] = wf_L*maxSpeed; // wL
  w[1] = wf_R*maxSpeed; // wR
}

// BLE Poll for this central: handles scanning for devices
bool connectedScan() {
  // Device connected. Procede function
  if(dev.connected()) {
    return true;
  }

  // Device not connected but was defined, indicates disconenction
  else if(dev) {
    Serial.println("Disconnected from Central: " + dev.localName());
    disconnectionLED();
    dev = BLE.available();
  }

  // No device found. Continue scanning
  else {
    if(!scanning) {
      BLE.scanForUuid(CART_UUID);
      scanning = true;
      Serial.println("Start Scanning...");
    }

    dev = BLE.available();
    if(dev) {
      Serial.println("Found: " + dev.localName());
      if(dev.localName() != "cart1") return false;
      BLE.stopScan();
      scanning = false;

      // Verify Connection
      if(!dev.connect()) {
        Serial.println("Failed to Connect");
        return false;
      }

      if(!dev.discoverAttributes()) {
        Serial.println("Failed to Discover Attributes");
        dev.disconnect();
        return false;
      }

      verifyCharacteristic(&sensorData, DATA_UUID);
      //verifyCharacteristic(&wheelRef, COMMAND_UUID);
      
      if(!sensorData.canSubscribe()) {
        Serial.println("Cannot subscribe");
        dev.disconnect();
        return false;
      }
      sensorData.subscribe();
      
      connectionLED();
      Serial.println("Connection Successful");
    }
  }

  return false;
}
// Function to verify BLE Characteristics
void verifyCharacteristic(BLECharacteristic *characteristic, char* UUID) {
  *characteristic = dev.characteristic(UUID);
  if (!*characteristic) {
    Serial.println("No Characteristic for UUID " + String(UUID));
    dev.disconnect();
    return;
  }
}

// LED Handlers: For Nano 33 BLE RGB LED, HIGH is off, LOW is on
void connectionLED() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
}
void disconnectionLED() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, HIGH);
}

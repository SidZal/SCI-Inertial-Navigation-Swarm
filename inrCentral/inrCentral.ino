#include <ArduinoBLE.h>
#include "Joystick.h"

#define MAX_RPM 255

// Bluetooth Objects
BLEService inrStation("b440");
BLECharacteristic omega("ba89", BLERead, 8);
BLECharacteristic inertialData("14df", BLEWrite | BLENotify, 16);
BLEDevice central;

// Joystick Objects
Joystick jsA(A0);
Joystick jsB(A1);

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // Calibrate Joysticks
  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());
  delay(500);

  jsA.setMax(255);
  jsB.setMax(255);

  Serial.println("Initialzing BLE Service");  
  // Initialize BLE Service
  if(!BLE.begin()) {
    Serial.println("Failed to start BLE");
    while(1);
  }
  Serial.println("BLE Initialized");

  BLE.setLocalName("receiver");
  BLE.setAdvertisedService(inrStation);

  inrStation.addCharacteristic(omega);
  inrStation.addCharacteristic(inertialData);

  BLE.addService(inrStation);
  BLE.advertise();
  Serial.println("Setup Complete");
}

// Handles bluetooth, calls receiverLoop in normal operation
void loop() {
  // Conencted
  if(central.connected()) {
    receiverLoop();
  }
  // Recently Disconnected
  else if (central) {
    Serial.println("Disconnected from Central: " + central.address());
    central = BLE.central();
  }
  // Scanning
  else {
    Serial.println("Advertising BLE Service");
    while(!central.connected())
      central = BLE.central();
    Serial.println("Connected to Central: " + central.address());
  }
}

void receiverLoop() {
  // Read quats
  float quats[4];
  inertialData.readValue(&quats, 16);
  Serial.println(String(quats[0]) + ' ' + String(quats[1]) + ' ' + String(quats[2]) + ' ' + String(quats[3]));

  // Write Joystick Inputs
  float Ain = jsA.read();
  float Bin = jsB.read();

  // omega factor: -1 to 1
  float wf_L = Ain + Bin;  
  float wf_R = Ain - Bin;
  if(abs(wf_L)>1)
    wf_L /= abs(wf_L);
  if(abs(wf_R)>1)
    wf_R /= abs(wf_R);

  int w[2];
  w[0] = wf_L*MAX_RPM; // wL
  w[1] = wf_R*MAX_RPM; // wR

  //Serial.println(String(w[0]) + ' ' + String(w[1]));
  omega.writeValue(&w, 8);
}
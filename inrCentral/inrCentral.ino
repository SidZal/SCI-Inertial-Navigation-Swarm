#include <ArduinoBLE.h>
#include "Joystick.h"

// Bluetooth Objects
BLEService inrStation("b440");
BLECharacteristic pwm("ba89", BLEWrite | BLERead, 8);
BLECharacteristic idle("14df", BLEWrite | BLERead | BLENotify | BLEIndicate, 8);
BLEDevice central;

// Joystick Objects
Joystick jsA(A0);
Joystick jsB(A1);
int idleVal[2];

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // Calibrate Joysticks
  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());
  idleVal[0] = jsA.getIdle();
  idleVal[1] = jsB.getIdle();
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

  inrStation.addCharacteristic(pwm);
  inrStation.addCharacteristic(idle);

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
    idle.writeValue(&idleVal, 8);
  }
}

void receiverLoop() {
  // Write Joystick Inputs
  int jsIn[2];
  jsIn[0] = jsA.read();
  jsIn[1] = jsB.read();
  //Serial.println(String(jsIn[0]) + ' ' + String(jsIn[1]));
  pwm.writeValue(&jsIn, 8);
}
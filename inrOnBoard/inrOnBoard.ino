#include "Motor.h"
#include <ArduinoBLE.h>

// Bluetooth UUIDs
#define PERIPHERAL_UUID "b440"
#define PWM_UUID "ba89"
#define IDLE_UUID "14df"

// Bluetooth Helper Variables
bool scanning = false;
BLEDevice peripheral;

BLECharacteristic pwm;
BLECharacteristic idle;

#define enA 11
#define in1 10
#define in2 12

#define enB 6
#define in3 7
#define in4 8

Motor motorA(in1, in2, enA, 2, 4, 0);
Motor motorB(in3, in4, enB, 3, 5, 0);

int idleVal[2];

void setup() {
  Serial.begin(9600);

  Serial.println("Initializing BLE");
  if(!BLE.begin()) {
    Serial.println("Failed to initialize BLE");
    while(1);
  }
  Serial.println("BLE Initialized");
}

void loop() {
  if(peripheral.connected()) {
    transmissionLoop();
  }
  else {

    if(!scanning) {
      BLE.scanForUuid(PERIPHERAL_UUID);
      scanning = true;
      Serial.println("Scanning...");
    }

    peripheral = BLE.available();
    if(peripheral) {
      Serial.println("Found: " + peripheral.localName());
      if (peripheral.localName() != "receiver") return;
      BLE.stopScan();
      scanning = false;

      // Verify Connection
      if(!peripheral.connect()) {
        Serial.println("Failed to Connect");
        return;
      }

      if(!peripheral.discoverAttributes()) {
        Serial.println("Failed to Discover Attributes");
        peripheral.disconnect();
        return;
      }

      verifyCharacteristic(&pwm, PWM_UUID);
      verifyCharacteristic(&idle, IDLE_UUID);

      Serial.println("Connection Successful");

      while(!idleVal[0]) {
        Serial.println("Waiting for idle");
        idle.readValue(&idleVal, 8);
      }
    }
  }
}

void transmissionLoop() {
  int inps[2];
  pwm.readValue(&inps, 8);
  float straight = (inps[0]-idleVal[0])/513.*255;
  float turn = (inps[1]-idleVal[1])/513.*255;

  motorA.drive(straight + turn);
  motorB.drive(straight - turn);
  Serial.println(String(inps[0]) + ' ' + String(inps[1]));
}

// Function to verify BLE Characteristics
void verifyCharacteristic(BLECharacteristic *characteristic, char* UUID) {
  *characteristic = peripheral.characteristic(UUID);
  if (!*characteristic) {
    Serial.println("No Characteristic for UUID " + String(UUID));
    peripheral.disconnect();
    return;
  } else if (!characteristic->canWrite()) {
    Serial.println("Cannot Write to Characteristic w/ UUID " + String(UUID));
    peripheral.disconnect();
    return;
  }
}
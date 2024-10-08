/*
  Base script for any given cart in swarm
  IDs like UUIDs change from cart to cart
*/

#include <ArduinoBLE.h>

// On-Board BLE Service
BLEService cartData("bac3");
BLECharacteristic sensorReadings("78d3", BLERead | BLENotify | BLEWrite, 36);
BLECharacteristic wheelRef("2bef", BLEWrite, 8);


void setup() {
  Serial.begin(115200);
  while(!Serial);

  setupBLE();
}

void loop() {
  BLE.poll();

  // Temp: put random floats into service
  float data[9] = {random(0,2), 2, 3, 4, 5, 6, 7, 8, random(0,2)};
  sensorReadings.writeValue(&data, 36);
}

// BLE Shorthands
void setupBLE() {
  Serial.println("Initializing BLE Service");

  if(!BLE.begin()) {
    Serial.println("Failed to initialize BLE");
    while(1);
  }

  BLE.setLocalName("cart1");
  BLE.setAdvertisedService(cartData);
  cartData.addCharacteristic(sensorReadings);
  cartData.addCharacteristic(wheelRef);
  BLE.addService(cartData);

  // Start LED disconnected
  disconnectionLED();

  // Set event handlers, to be called if event occurs during BLE.poll()
  BLE.setEventHandler(BLEConnected, cartConnected);
  BLE.setEventHandler(BLEDisconnected, cartDisconnected);
  wheelRef.setEventHandler(BLEWritten, wheelHandler);

  BLE.advertise();
  Serial.println("BLE Setup Complete");
}

// BLE Event Handlers
void cartConnected(BLEDevice dev) {
  Serial.println("Connected from " + dev.address());
  connectionLED();
}
void cartDisconnected(BLEDevice dev) {
  Serial.println("Disconnected from " + dev.address());
  disconnectionLED();
}
void wheelHandler(BLEDevice dev, BLECharacteristic characteristic) {
  int omega[2];
  wheelRef.readValue(&omega, 8);
  Serial.println(String(omega[0]) + " " + String(omega[1]));
}


// LED Handlers: For Nano 33 BLE, HIGH is off, LOW is on
void connectionLED() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
}
void disconnectionLED() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, HIGH);
}

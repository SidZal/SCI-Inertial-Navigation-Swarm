#include <ArduinoBLE.h>
#include "Joystick.h"

#define MAX_RPM 255
#define RAW_DATA_SCALE 16384

// Bluetooth Objects
BLEService inrStation("b440");
BLECharacteristic omega("ba89", BLERead, 8);
BLECharacteristic dmpData("14df", BLEWrite | BLENotify, 28);
BLECharacteristic rawData("70ce", BLEWrite | BLENotify, 12);
BLEDevice central;

// Command Tracker
unsigned long cmdTimer = 0;
unsigned long lastLoop = 0;

// Joystick Objects
Joystick jsA(A0);
Joystick jsB(A1);

// Omega Array
int w[2];

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.setTimeout(1);

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
  inrStation.addCharacteristic(dmpData);
  inrStation.addCharacteristic(rawData);

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
  unsigned long loopTime = millis(); // Remember current loop's time
  readData();

  if(cmdTimer > 0) {
    unsigned long sinceLast = loopTime - lastLoop;
    if (sinceLast > cmdTimer)
      cmdTimer = 0;
    else
      cmdTimer -= sinceLast;
  }
  else {
    String serCmd = Serial.readStringUntil(' ');
    int param = Serial.readStringUntil('\n').toInt();
    if(param>100) param = 100;
    
    if(serCmd == "circle") {
      w[0] = 255;
      w[1] = -255*param/100;
      cmdTimer = 5000;
    } 
    else {
      JoystickControl(w);
    }
  }

  omega.writeValue(&w, 8);

  lastLoop = loopTime; // global record of loop time
}

// reads joystick inputs, converts to wL and wR
void JoystickControl(int* w) {
  float Ain = jsA.read();
  float Bin = jsB.read();

  // omega factor: -1 to 1
  float wf_L = Ain + Bin;  
  float wf_R = Ain - Bin;
  if(abs(wf_L)>1)
    wf_L /= abs(wf_L);
  if(abs(wf_R)>1)
    wf_R /= abs(wf_R);

  w[0] = wf_L*MAX_RPM; // wL
  w[1] = wf_R*MAX_RPM; // wR
}

// main loop function that reads and processes data from peripheral
void readData() {
  // Read quats
  float quatsypr[7];
  dmpData.readValue(&quatsypr, 28);

  int16_t rawMotion[6];
  float rawC[6]; // raw data Converted
  rawData.readValue(&rawMotion, 12);
  for(int rawI = 0; rawI<6; rawI++)
    rawC[rawI] = 9.81 * (float)rawMotion[rawI]/RAW_DATA_SCALE;

  // Raw Accel, raw gyro, quats, ypr
  Serial.println(printArray(rawC, 6) + ' ' + printArray(quatsypr, 7));
}

// helper function prints float array of given length
String printArray(float* arr, int n) {
  if(n<1) return "";
  String result = String(arr[0]);
  for(int i = 1; i<n; i++)
    result += ' ' + String(arr[i]);
  return result;
}
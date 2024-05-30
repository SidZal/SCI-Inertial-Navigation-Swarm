#include <ArduinoBLE.h>
#include "Joystick.h"

// Constants
#define MAX_RPM 150
#define RAW_DATA_SCALE 16384
#define pi 3.1415926

// Bluetooth UUIDs
#define DEV_UUID "b440"
#define OMEGA_UUID "ba89"
#define OMEGADATA_UUID "c81a"
#define DMPDATA_UUID "14df"
#define RAWDATA_UUID "70ce"
#define HEADINGCONTROL_UUID "289b"
#define PIDTUNE_UUID "0ef0"

BLEDevice dev;
bool scanning = false;

BLECharacteristic omega;
BLECharacteristic omegaData;
BLECharacteristic dmpData;
BLECharacteristic rawData;
BLECharacteristic headingControl;
BLECharacteristic PIDtune;

// Command Tracker
unsigned long cmdTimer = 0;
unsigned long lastLoop = 0;
String serCmd;
float param1, param2, param3;
float integrator;
float previousE;

// Joystick Objects
Joystick jsA(A0);
Joystick jsB(A1);

// Omega Array
int w[2];

// Print debugging information
bool debugger = true;

// IMU Data
float quatsypr[7];
float rawC[6];

String last;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.setTimeout(1);

  // Calibrate Joysticks
  if(debugger) {
    Serial.println(jsA.calibrate());
    Serial.println(jsB.calibrate());
    delay(500);
  }
  else {
    jsA.calibrate();
    jsB.calibrate();
  }

  jsA.setMax(255);
  jsB.setMax(255);

  if(debugger) Serial.println("Initialzing BLE Service");  
  if(!BLE.begin()) {
    if(debugger) Serial.println("Failed to start BLE");
    while(1);
  }
  if(debugger) Serial.println("BLE Initialized");

  digitalWrite(LEDR, LOW); // low is on, high is off
  digitalWrite(LEDB, HIGH);

  // Heading controller intiialization
  float href[2]; href[0] = 0;
  headingControl.writeValue(&href, 8);
}

// Handles bluetooth, calls receiverLoop in normal operation
void loop() {
  if(dev.connected()) {
    receiverLoop();
  }
  else if (dev) {
    Serial.println("Disconnected from Central: " + dev.localName());
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDB, HIGH);
    dev = BLE.available();
  }
  else {
    if(!scanning) {
      BLE.scanForUuid(DEV_UUID);
      scanning = true;
      Serial.println("Scanning...");
    }

    dev = BLE.available();
    if(dev) {
      Serial.println("Found: " + dev.localName());
      if (dev.localName() != "INR1") return;
      BLE.stopScan();
      scanning = false;

      // Verify Connection
      if(!dev.connect()) {
        Serial.println("Failed to Connect");
        return;
      }

      if(!dev.discoverAttributes()) {
        Serial.println("Failed to Discover Attributes");
        dev.disconnect();
        return;
      }

      verifyCharacteristic(&omega, OMEGA_UUID);
      verifyCharacteristic(&omegaData, OMEGADATA_UUID);
      verifyCharacteristic(&dmpData, DMPDATA_UUID);
      verifyCharacteristic(&rawData, RAWDATA_UUID);
      verifyCharacteristic(&headingControl, HEADINGCONTROL_UUID);
      verifyCharacteristic(&PIDtune, PIDTUNE_UUID);
      
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDB, LOW);
      Serial.println("Connection Successful");
      

      lastLoop = millis();
    }
  }
}

void receiverLoop() {
  readData(); // Read and print IMU data to serial

  // send joystick
  JoystickControl(w);
  omega.writeValue(&w, 8);
}

// new serial commands entered here
void serialCommandControl(unsigned long dTmillis, int* w, String serCmd, float cmdParam1, float cmdParam2, float cmdParam3, bool init, bool fnl) {
  double dT = dTmillis/1000.;

  if(serCmd == "circle") {
    if(init) cmdTimer = 5000;
    w[0] = 255;
    w[1] = -255*cmdParam1/10;
  }
  else if(serCmd == "heading") {
    if(init) {cmdTimer = 10000; integrator = 0; previousE = 0;}
    float href[2];
    if(fnl) href[0] = 0; else href[0] = 1;
    href[1] = cmdParam1;
    headingControl.writeValue(&href, 8);
    /*
    float e_deg = cmdParam1 - quatsypr[4]*180/3.1415;
    integrator += e_deg*dT;
    float e_der = (e_deg-previousE)/dT;
    previousE = e_deg;
    int controlIn = .5*e_deg + .75*integrator + 5*e_der;
    Serial.println(String(e_deg) + ' ' + String(integrator) + ' ' + String(e_der) + ' ' + String(controlIn));
    w[0] = -controlIn;
    w[1] = -controlIn;
    */
  }
  else if(serCmd == "sineHeading") {
    if(init) cmdTimer = 10000;
    int PWMin = 10*(2*3.1415*sin(10-cmdTimer/1000.) - quatsypr[4]);
    Serial.println(PWMin);
    w[0] = PWMin;
    w[1] = PWMin;
  }
  else if(serCmd == "PID") {
    float newPID[3]; newPID[0] = cmdParam1; newPID[1] = cmdParam2; newPID[2] = cmdParam3;
    PIDtune.writeValue(&newPID, 12);
  }
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
  // Read quats, ypr
  dmpData.readValue(&quatsypr, 28);

  int16_t rawMotion[6];
  rawData.readValue(&rawMotion, 12);
  for(int rawI = 0; rawI<6; rawI++)
    rawC[rawI] = 9.81 * (float)rawMotion[rawI]/RAW_DATA_SCALE;

  float wheelVelocity[2];
  omegaData.readValue(&wheelVelocity, 8);

  // Raw Accel, raw gyro, quats, ypr, omega from enc
  String output = printArray(rawC, 6) + ", " + printArray(quatsypr, 7) + ", " + printArray(wheelVelocity, 2);
  Serial.println(output);
}

// helper function prints float array of given length
String printArray(float* arr, int n) {
  if(n<1) return "";
  String result = String(arr[0]);
  for(int i = 1; i<n; i++)
    result += ", " + String(arr[i]);
  return result;
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
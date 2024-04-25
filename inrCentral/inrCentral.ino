#include <ArduinoBLE.h>
#include "Joystick.h"

#define MAX_RPM 255
#define RAW_DATA_SCALE 16384

#define RED 22     
#define BLUE 24     

#define pi 3.1415926

// Bluetooth Objects
BLEService inrStation("b440");
BLECharacteristic omega("ba89", BLERead, 8);
BLECharacteristic dmpData("14df", BLEWrite | BLENotify | BLEIndicate, 28);
BLECharacteristic rawData("70ce", BLEWrite | BLENotify | BLEIndicate, 12);
BLECharacteristic headingControl("289b", BLERead, 8);
BLECharacteristic PIDtune("0ef0", BLERead, 12);
BLEDevice central;

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

void setup() {
  Serial.begin(9600);
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
  // Initialize BLE Service
  if(!BLE.begin()) {
    if(debugger) Serial.println("Failed to start BLE");
    while(1);
  }
  if(debugger) Serial.println("BLE Initialized");

  BLE.setLocalName("receiver");
  BLE.setAdvertisedService(inrStation);

  inrStation.addCharacteristic(omega);
  inrStation.addCharacteristic(dmpData);
  inrStation.addCharacteristic(rawData);
  inrStation.addCharacteristic(headingControl);
  inrStation.addCharacteristic(PIDtune);

  BLE.addService(inrStation);
  BLE.advertise();
  if(debugger) Serial.println("Setup Complete");

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);

  digitalWrite(RED, LOW); // low is on, high is off
  digitalWrite(BLUE, HIGH);

  float href[2]; href[0] = 0;
  headingControl.writeValue(&href, 8);
}

// Handles bluetooth, calls receiverLoop in normal operation
void loop() {
  // Conencted
  if(central.connected()) {
    receiverLoop();
  }
  // Recently Disconnected
  else if (central) {
    if(debugger) Serial.println("Disconnected from Central: " + central.address());
    central = BLE.central();
    digitalWrite(BLUE, HIGH);
    digitalWrite(RED, LOW);
  }
  // Scanning
  else {
    if(debugger) Serial.println("Advertising BLE Service");
    while(!central.connected())
      central = BLE.central();
    if(debugger) Serial.println("Connected to Central: " + central.address());
    digitalWrite(BLUE, LOW);
    digitalWrite(RED, HIGH);
  }
}

void receiverLoop() {
  unsigned long loopTime = millis(); // Remember current loop's time
  unsigned long dT = loopTime - lastLoop;
  readData(); // Read and print IMU data to serial

  // Check for active command
  if(cmdTimer > 0) {


    if (dT > cmdTimer) {
      serialCommandControl(dT, w, serCmd, param1, param2, param3, false, true);
      cmdTimer = 0;
      param1 = param2 = param3 = 0;
    }
    else {
      cmdTimer -= dT;
      serialCommandControl(dT, w, serCmd, param1, param2, param3, false, false);
    }
  }
  else if (Serial.available()) {

    // Read in serial command
    serCmd = Serial.readStringUntil(' ');
    param1 = Serial.readStringUntil(' ').toFloat();
    param2 = Serial.readStringUntil(' ').toFloat();
    param3 = Serial.readStringUntil('\n').toFloat();
    serialCommandControl(dT, w, serCmd, param1, param2, param3, true, false);

  }
  else // Default to joystick control
    JoystickControl(w);

  omega.writeValue(&w, 8);

  lastLoop = loopTime; // global record of loop time
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
  if(1) {
    // Read quats
    //float prevHeading = quatsypr[4];
    dmpData.readValue(&quatsypr, 28);
    //quatsypr[4] = unwrap(prevHeading, quatsypr[4]);

    int16_t rawMotion[6];
    rawData.readValue(&rawMotion, 12);
    for(int rawI = 0; rawI<6; rawI++)
      rawC[rawI] = 9.81 * (float)rawMotion[rawI]/RAW_DATA_SCALE;

    // Raw Accel, raw gyro, quats, ypr
    Serial.println(printArray(rawC, 6) + ", " + printArray(quatsypr, 7));
  }
}

// helper function prints float array of given length
String printArray(float* arr, int n) {
  if(n<1) return "";
  String result = String(arr[0]);
  for(int i = 1; i<n; i++)
    result += ", " + String(arr[i]);
  return result;
}
#include "Motor.h"
#include "rk4.h"
#include "dualWheelModel.h"
#include "utility.h"
#include <ArduinoBLE.h>

// Bluetooth UUIDs
#define PERIPHERAL_UUID "b440"
#define PWM_UUID "ba89"
#define IDLE_UUID "14df"

#define MAX_RPM 25

// Bluetooth Helper Variables
bool scanning = false;
BLEDevice peripheral;

BLECharacteristic pwm;
BLECharacteristic idle;

// Joystick Idle Values
int idleVal[2];

// Motor + Encoder Pins
#define PWMA 6
#define in1 8
#define in2 7
#define C1A 3
#define C2A 5

#define MOTOR_STANDBY 9

#define PWMB 11
#define in3 10
#define in4 12
#define C1B 2
#define C2B 4

#define pi 3.1415926

// Low-Level Speed Control Parameters
const float CPR = 12;
const float gearRatio = 100.37;
const float conversionRatio = 4./CPR*60/gearRatio;
Motor motorA(in1, in2, PWMA, C1A, C2A, conversionRatio);
Motor motorB(in3, in4, PWMB, C1B, C2B, conversionRatio);

float v[2];

// Orientation Controller Variables
int n = 3; // State Variables
double t0 = 0, t1;
double X0[3], X1[3];
double wL = 0, wR = 0;
double kp = .05;
double wMax = 10, wComm, wInput;
double error;
unsigned long millisLast = 0;
double theta_d=0;

void setup() {
  Serial.begin(9600);
  X0[0] = X0[1] = X0[2] = 0.0;
  attachInterrupt(digitalPinToInterrupt(C1A), motorAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(C1B), motorBInterrupt, RISING);

  digitalWrite(MOTOR_STANDBY, HIGH);

  motorA.tunePID(1, 4, 0);
  motorB.tunePID(1, 4, 0);

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
  else if (peripheral) {
    Serial.println("Disconnected from Peripheral: " + peripheral.localName());
    peripheral = BLE.available();
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

      millisLast = millis();
    }
  }
}

void transmissionLoop() {
  double dt = (millis() - millisLast)/1000.0;
  if(dt < 0.1)
    return;
  millisLast = millis();

  int inps[2];
  pwm.readValue(&inps, 8);

  float jsA = (inps[0]-idleVal[0])/513.;
  float jsB = (inps[1]-idleVal[1])/513.;

  if(pow(jsA,2) + pow(jsB,2) > 0.8) 
    theta_d = -atan2(jsA, jsB)*180/pi;

  error = theta_d - X0[2]*180/pi;
  wComm = speedSaturator(kp*error, wMax);
  wInput = -wComm * 60 / (2*pi);
  
  t1 = t0 + dt;
  
  rk4(t0, n, X0, X1, dt, v[0]*2*pi/60, v[1]*2*pi/60, dualWheelModel);
  //rk4(t0, n, X0, X1, dt, -wComm, wComm, dualWheelModel);

  t0 = t1;
  for(int i=0; i<n; i++)
    X0[i] = X1[i];

  float Aref = jsA + jsB;
  if(abs(Aref)>1)
    Aref /= abs(Aref);
  float Bref = jsA - jsB;
  if(abs(Bref)>1)
    Bref /= abs(Bref);

  v[0] = motorA.feedbackStep(wInput);
  v[1] = -motorB.feedbackStep(wInput);
  Serial.println(String(dt) + ' ' + String(theta_d) + ' '  + String(error) + ' ' + String(X1[2]*180/pi) + ' ' + String(wInput) + ' ' + String(v[0]) + ' ' + String(v[1]));
  //Serial.println(String(MAX_RPM) + ' ' + String(-MAX_RPM) + ' ' + String(v[0]) + ' ' + String(v[1]) + ' ' + String(Aref*MAX_RPM) + ' ' + String(Bref *MAX_RPM));
  //delay(50);
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

// Motor Interrupts from Encoders
void motorAInterrupt() {
  motorA.readEncoder();
}
void motorBInterrupt() {
  motorB.readEncoder();
}
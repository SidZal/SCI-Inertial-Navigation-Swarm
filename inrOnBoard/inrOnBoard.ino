#include "Motor.h"
#include "rk4.h"
#include "dualWheelModel.h"
#include "utility.h"
#include <ArduinoBLE.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Bluetooth UUIDs
#define PERIPHERAL_UUID "b440"
#define UPDATED_UUID "cf1a"
#define OMEGA_UUID "ba89"
#define OMEGADATA_UUID "c81a"
#define DMPDATA_UUID "14df"
#define RAWDATA_UUID "70ce"
#define HEADINGCONTROL_UUID "289b"
#define PIDTUNE_UUID "0ef0"

// IMU Setup
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float quatsypr[7];
float orientation[3];
int16_t motion6[6];

// Bluetooth Helper Variables
bool scanning = false;
BLEDevice peripheral;

BLECharacteristic updated;
BLECharacteristic omega;
BLECharacteristic omegaData;
BLECharacteristic dmpData;
BLECharacteristic rawData;
BLECharacteristic headingControl;
BLECharacteristic PIDtune;

// Motor + Encoder Pins
#define PWMA 6
#define in1 8
#define in2 7
#define C1A 3
#define C2A 5

#define MOTOR_STANDBY 9

#define PWMB 12
#define in3 11
#define in4 10
#define C1B 2
#define C2B 4

#define pi 3.1415926

#define MAX_RPM 100

// Low-Level Speed Control Parameters
const float countsPerRotation = 3;
const float gearRatio = 298;
const float conversionRatio = 1./countsPerRotation*60/gearRatio;
Motor motorA(in1, in2, PWMA, C1A, C2A, conversionRatio);
Motor motorB(in3, in4, PWMB, C1B, C2B, conversionRatio);

float v[2];

// Orientation Controller Variables
/*
int n = 3; // State Variables
double t0 = 0, t1;
double X0[3], X1[3];
double kp = .05;
double wMax = 10, wComm, wInput;
double orientationError;
double theta_d=0;
*/
unsigned long millisLast = 0;
float wL = 0, wR = 0;
float integrator;
float e_prev;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);

  IMUsetup();

  //X0[0] = X0[1] = X0[2] = 0.0;
  attachInterrupt(digitalPinToInterrupt(C1A), motorAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(C1B), motorBInterrupt, RISING);

  digitalWrite(MOTOR_STANDBY, HIGH);

  motorA.tunePID(1.5, 1, 0);
  motorB.tunePID(1.5, 1, 0);

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

      verifyCharacteristic(&updated, UPDATED_UUID);
      verifyCharacteristic(&omega, OMEGA_UUID);
      verifyCharacteristic(&omegaData, OMEGADATA_UUID);
      verifyCharacteristic(&dmpData, DMPDATA_UUID);
      verifyCharacteristic(&rawData, RAWDATA_UUID);
      verifyCharacteristic(&headingControl, HEADINGCONTROL_UUID);
      verifyCharacteristic(&PIDtune, PIDTUNE_UUID);

      Serial.println("Connection Successful");

      millisLast = millis();
    }
  }
}

void transmissionLoop() {
  readIMU();

  /*
  float pidks[3];
  PIDtune.readValue(&pidks,12);

  double dt = (millis() - millisLast)/1000.0;
  if(dt < 0.05)
    return;
  millisLast = millis();

  float href[2];
  headingControl.readValue(&href, 8);
  
  if(href[0]) {
    float e_deg = href[1] - quatsypr[4] * 180/pi;
    integrator += e_deg*dt;
    float e_der = (e_deg-e_prev)/dt;
    e_prev = e_deg;
    int controlIn = pidks[0]*e_deg + pidks[1]*integrator + pidks[2]*e_der;
    Serial.println(String(dt) + ' ' + String(pidks[0]) + ' ' + String(e_deg) + ' ' + String(pidks[1]) + ' ' + String(integrator) + ' ' + String(pidks[2]) + ' ' + String(e_der) + ' ' + String(quatsypr[4]));
    wL = -controlIn; wR = -controlIn;
  }
  else {
    if(integrator != 0) {integrator = 0; e_prev = 0;}

    int w[2];
    omega.readValue(&w, 8);
    wL = w[0]; wR = w[1];
  }
  */

  /*
  if(pow(jsA,2) + pow(jsB,2) > 0.8) 
    theta_d = -atan2(jsA, jsB)*180/pi;

  orientationError = theta_d - X0[2]*180/pi;
  wComm = speedSaturator(kp*orientationError, wMax);
  wInput = -wComm * 60 / (2*pi);
  
  t1 = t0 + dt;
  
  rk4(t0, n, X0, X1, dt, v[0]*2*pi/60, v[1]*2*pi/60, dualWheelModel);
  //rk4(t0, n, X0, X1, dt, -wComm, wComm, dualWheelModel);

  t0 = t1;
  for(int i=0; i<n; i++)
    X0[i] = X1[i];
  */

  int w[2];
  omega.readValue(&w, 8);
  wL = w[0]; wR = w[1];
  v[0] = motorA.feedbackStep(wL);
  v[1] = motorB.feedbackStep(wR);
  //v[0] = motorA.feedbackStep(Aref*MAX_RPM);
  //v[1] = -motorB.feedbackStep(Bref*MAX_RPM);
  //Serial.println(String(dt) + ' ' + String(theta_d) + ' '  + String(orientationError) + ' ' + String(X1[2]*180/pi) + ' ' + String(wInput) + ' ' + String(v[0]) + ' ' + String(v[1]));
  Serial.println(String(MAX_RPM) + ' ' + String(-MAX_RPM) + ' ' + String(v[0]) + ' ' + String(v[1]) + ' ' + String(wL) + ' ' + String(wR));
  //Serial.println(String(dt) + ' ' + String(motion6[0]) + ' ' + String(motion6[1]) + ' ' + String(motion6[2]) + ' ' + String(motion6[3]) + ' ' + String(motion6[4]) + ' ' + String(motion6[5]));
  //delay(50);
}

// Function to verify BLE Characteristics
void verifyCharacteristic(BLECharacteristic *characteristic, char* UUID) {
  *characteristic = peripheral.characteristic(UUID);
  if (!*characteristic) {
    Serial.println("No Characteristic for UUID " + String(UUID));
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

void readIMU() {
  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    float prevHeading = quatsypr[4];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(quatsypr+4, &q, &gravity);
    mpu.getMotion6(motion6, motion6+1, motion6+2, motion6+3, motion6+4, motion6+5);

    quatsypr[4] = unwrap(prevHeading, quatsypr[4]);
    orientation[0] = quatsypr[4] * 180/M_PI;
    orientation[1] = quatsypr[5] * 180/M_PI;
    orientation[2] = quatsypr[6] * 180/M_PI;

    quatsypr[0] = q.w; quatsypr[1] = q.x; quatsypr[2] = q.y; quatsypr[3] = q.z;
    dmpData.writeValue(&quatsypr, 28);
    rawData.writeValue(&motion6, 12);
    
    bool update = true;
    updated.writeValue(&update, 1);
  }
  omegaData.writeValue(&v, 8);
}

// helper function to unwrap two angles
float unwrap(float prev, float next) {
  while(abs(prev-next) > pi)
    if(next>prev)
      next -= 2*pi;
    else
      next += 2*pi;

  return next;
}

void IMUsetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}
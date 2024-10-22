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

//
// Bluetooth Variables
//
// BLE Service Setup
BLEService inrService("8f2f");
// Characteristic for sending wL/wR to robot
BLECharacteristic omega("ba89", BLEWrite, 8);

// Characteristics for receiving data from Robot
BLECharacteristic botData("c81a", BLERead, 36);

// Special mode characteristics
BLECharacteristic PIDtune("0ef0", BLEWrite, 12);

//
// IMU Variables
//
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float quatsypr[7];
float orientation[3];
int16_t motion6[6];

//
// Motor Speed Control Variables
//
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
#define RAW_DATA_SCALE 16384
#define GRAVITATIONAL_ACCEL 9.81
#define MAX_RPM 100

// Low-Level Speed Control Parameters
const float countsPerRotation = 3;
const float gearRatio = 298;
const float conversionRatio = 1./countsPerRotation*60/gearRatio;
Motor motorA(in1, in2, PWMA, C1A, C2A, conversionRatio);
Motor motorB(in3, in4, PWMB, C1B, C2B, conversionRatio);

float v[2];

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

  Serial.begin(115200);
  //while(!Serial);

  IMUsetup();

  attachInterrupt(digitalPinToInterrupt(C1A), motorAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(C1B), motorBInterrupt, RISING);

  digitalWrite(MOTOR_STANDBY, HIGH);

  motorA.tunePID(5, 1, 0);
  motorB.tunePID(5, 1, 0);

  Serial.println("Initializing BLE Service");
  if(!BLE.begin()) {
    Serial.println("Failed to initialize BLE");
    while(1);
  }
  Serial.println("BLE Initialized");
  Serial.println(BLE.address());

  BLE.setLocalName("INR1");
  BLE.setAdvertisedService(inrService);

  inrService.addCharacteristic(omega);
  inrService.addCharacteristic(botData);
  inrService.addCharacteristic(PIDtune);

  BLE.addService(inrService);
  BLE.advertise();
  Serial.println("Setup Complete");

  BLE.setEventHandler(BLEConnected, inrConnect);
  BLE.setEventHandler(BLEDisconnected, inrDisconnect);
  omega.setEventHandler(BLEWritten, setSpeedFromBLE);
  PIDtune.setEventHandler(BLEWritten, setPID);

  // Red light -> no connection, blue light -> connection
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, HIGH);
}

void loop() {
  readIMU();
  BLE.poll();
  
  v[0] = motorA.feedbackStep(wL);
  v[1] = motorB.feedbackStep(wR);
  //Serial.println(String(MAX_RPM) + ' ' + String(-MAX_RPM) + ' ' + String(v[0]) + ' ' + String(v[1]) + ' ' + String(wL) + ' ' + String(wR));
  //Serial.println(String(dt) + ' ' + String(theta_d) + ' '  + String(orientationError) + ' ' + String(X1[2]*180/pi) + ' ' + String(wInput) + ' ' + String(v[0]) + ' ' + String(v[1]));
  //Serial.println(String(MAX_RPM) + ' ' + String(-MAX_RPM) + ' ' + String(v[0]) + ' ' + String(v[1]) + ' ' + String(wL) + ' ' + String(wR));
  //Serial.println(String(dt) + ' ' + String(motion6[0]) + ' ' + String(motion6[1]) + ' ' + String(motion6[2]) + ' ' + String(motion6[3]) + ' ' + String(motion6[4]) + ' ' + String(motion6[5]));
}

// Motor Interrupts from Encoders
void motorAInterrupt() {
  motorA.readEncoder();
}
void motorBInterrupt() {
  motorB.readEncoder();
}

// Event Handler for Disconnection
void inrDisconnect(BLEDevice dev) {
  Serial.println("Disconnected from " + dev.localName() + " at " + dev.address());
  wL = 0; wR = 0;
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, HIGH);
}

// Event Handler for Connection
void inrConnect(BLEDevice dev) {
  Serial.println("Connected to " + dev.localName() + " at " + dev.address());
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
}

// Event handler for setting wheel speed
void setSpeedFromBLE(BLEDevice dev, BLECharacteristic omega) {
  int w[2];
  omega.readValue(&w, 8);
  wL = -w[0]; wR = w[1];
}

void setPID(BLEDevice dev, BLECharacteristic pid) {
  float pid_ks[3];
  pid.readValue(&pid_ks, 12);
  Serial.println(String(pid_ks[0]) + '\t' + String(pid_ks[1]) + '\t' + String(pid_ks[2]));

  motorA.tunePID(pid_ks[0], pid_ks[1], pid_ks[2]);
  motorB.tunePID(pid_ks[0], pid_ks[1], pid_ks[2]);
}

void readIMU() {
  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    float prevHeading = quatsypr[4];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(quatsypr+4, &q, &gravity);

    mpu.getMotion6(motion6, motion6+1, motion6+2, motion6+3, motion6+4, motion6+5);

    float robotData[9];
    for(int i=0; i<6; i++)
      robotData[i] = (float)motion6[i]/RAW_DATA_SCALE * GRAVITATIONAL_ACCEL;

    quatsypr[4] = unwrap(prevHeading, quatsypr[4]);
    robotData[6] = quatsypr[4];

    robotData[7] = v[0];
    robotData[8] = v[1];

    //quatsypr[0] = q.w; quatsypr[1] = q.x; quatsypr[2] = q.y; quatsypr[3] = q.z;
    botData.writeValue(&robotData, 36);
    
    if(0) {
      //Serial.print(String(q.w) + ' ' + String(q.x) + ' ' + String(q.y) + ' ' + String(q.z) + ' ');
      //Serial.print(String(quatsypr[4]) + ' ' + String(quatsypr[5]) + ' ' + String(quatsypr[6]) )+ ' '
      //Serial.print(String(motion6[0]) + ' ' + String(motion6[1]) + ' ' + String(motion6[2]) + ' ');
      //Serial.println(String(motion6[3]) + ' ' + String(motion6[4]) + ' ' + String(motion6[5]));
    }
  }
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
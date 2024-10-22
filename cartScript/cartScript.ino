/*
  Base script for any given cart in swarm
  IDs like UUIDs change from cart to cart
*/
#define pi 3.1415926
#define RAW_DATA_SCALE 16384
#define GRAVITATIONAL_ACCEL 9.81

#include <ArduinoBLE.h>
#include "Motor.h"

// IMU Libraries
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// On-Board BLE Service
BLEService cartData("bac3");
BLECharacteristic sensorReadings("78d3", BLERead | BLENotify | BLEWrite, 36);
BLECharacteristic wheelRef("2bef", BLEWrite | BLEWriteWithoutResponse, 8);
const int ledPin = LED_BUILTIN; // pin to use for the LED

// IMU Variables
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float quatsypr[7];
float orientation[3];
int16_t motion6[6];


// Motor Pins
#define PWMA 6
#define in1 7
#define in2 8
#define C1A 3
#define C2A 5

#define MOTOR_STANDBY 9

#define PWMB 12
#define in3 11
#define in4 10
#define C1B 2
#define C2B 4

// Low-Level Speed Control Parameters
#define MAX_RPM 30
const float countsPerRotation = 3;
const float gearRatio = 1000;
const float conversionRatio = 1./countsPerRotation*60/gearRatio;
float kp[2],ki[2],kd[2];
Motor motorA(in1, in2, PWMA, C1A, C2A, conversionRatio);
Motor motorB(in3, in4, PWMB, C1B, C2B, conversionRatio);
float wheelVelocity[2];
int wheelVelocityTarget[2];


void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Motor setup: interrupts for encoders, standby turns on motor controller
  attachInterrupt(digitalPinToInterrupt(C1A), motorAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(C1B), motorBInterrupt, RISING);

  // Set default motor tunes (PER MOTOR BASIS)
  digitalWrite(MOTOR_STANDBY, HIGH);
  kp[0]=30; ki[0]=150; kd[0]=0.3;
  kp[1]=0; ki[1]=0; kd[1]=0;
  motorA.tunePID(kp[0],ki[0],kd[0]);
  motorB.tunePID(kp[1],ki[1],kd[1]);

  setupIMU();
  setupBLE();
}

void loop() {
  // Poll: waits for BLE events (connection/disconnection/write)
  BLE.poll();

  // float noise[9];
  // for (int i=0; i<9; i++) {
  //   noise[i] = (float) (10*random());
  // }
  // sensorReadings.writeValue(noise, 36);

  if (Serial.available() > 0)
    serialTuner();

  // TEMP: On-board Predetermined Control
  int cycle = 10000;
  int rest = 2000;
  int time = millis() % cycle;
  
  int targetVal = 15;
  if(time < cycle/2 - rest) {
    wheelVelocityTarget[0] = -targetVal; // L
    wheelVelocityTarget[1] = targetVal; // R
  }
  else if (time < cycle/2){
    wheelVelocityTarget[0] = 0; // L
    wheelVelocityTarget[1] = 0; // R
  }
  else if (time < cycle-rest){
    wheelVelocityTarget[0] = targetVal; // L
    wheelVelocityTarget[1] = -targetVal; // R
  }
  else {
    wheelVelocityTarget[0] = 0; // L
    wheelVelocityTarget[1] = 0; // R
  }
  
  // motorA.drive(wheelVelocityTarget[0]);
  // motorB.drive(wheelVelocityTarget[1]);

  // Motor control
  wheelVelocity[0] = motorA.feedbackStep(wheelVelocityTarget[0]);
  wheelVelocity[1] = motorB.feedbackStep(wheelVelocityTarget[1]);

  Serial.println(String(wheelVelocityTarget[0])+'\t'+String(wheelVelocity[0])+'\t'+String(wheelVelocityTarget[1])+'\t'+ String(wheelVelocity[1])+'\t'+"35  -35");

  readIMU();
}

// Serial Menu for Tuning
void serialTuner() {
  Serial.println("Stopping...");
  motorA.setPWM(0);
  motorB.setPWM(0);

  String userInput = Serial.readStringUntil('\n');
  if (userInput == "0" || userInput == "1") {
    int mtr = userInput.toInt();
    Serial.println("Motor " +userInput+ " Tune: " +String(kp[mtr])+ "\t" +String(ki[mtr])+ "\t" +String(kd[mtr]));
    kp[mtr] = serialTunerInputHelper("k_p");
    ki[mtr] = serialTunerInputHelper("k_i");
    kd[mtr] = serialTunerInputHelper("k_d");

    if(mtr) motorB.tunePID(kp[1], ki[1], kd[1]);
    else motorA.tunePID(kp[0], ki[0], kd[0]);
  }

  Serial.println("Continuing...");
  delay(1000);
  motorA.initFeedback();
  motorB.initFeedback();
}

float serialTunerInputHelper(String prop) {
  Serial.print("Please enter new (" + prop + "): ");
  while(Serial.available() == 0);
  String helperInput = Serial.readStringUntil('\n');
  Serial.println(helperInput.toFloat());
  return helperInput.toFloat();
}


// BLE Shorthands
void setupBLE() {
  Serial.println("Initializing BLE Service");

  // set LED pin to output mode: off = no connection
  pinMode(ledPin, OUTPUT);

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
  digitalWrite(ledPin, LOW);  

  // Set event handlers, to be called if event occurs during BLE.poll()
  BLE.setEventHandler(BLEConnected, cartConnected);
  BLE.setEventHandler(BLEDisconnected, cartDisconnected);
  wheelRef.setEventHandler(BLEWritten, wheelHandler);

  BLE.advertise();
  Serial.println(BLE.address());
  Serial.println("BLE Setup Complete");
}

void setupIMU() {
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

    robotData[7] = wheelVelocity[0];
    robotData[8] = wheelVelocity[1];

    //quatsypr[0] = q.w; quatsypr[1] = q.x; quatsypr[2] = q.y; quatsypr[3] = q.z;
    sensorReadings.writeValue(&robotData, 36);
    
    // Serial.print(String(q.w) + ' ' + String(q.x) + ' ' + String(q.y) + ' ' + String(q.z) + ' ');
    // Serial.print(String(quatsypr[4]) + ' ' + String(quatsypr[5]) + ' ' + String(quatsypr[6]) + ' ');
    // Serial.print(String(motion6[0]) + ' ' + String(motion6[1]) + ' ' + String(motion6[2]) + ' ');
    // Serial.println(String(motion6[3]) + ' ' + String(motion6[4]) + ' ' + String(motion6[5]));
  }
}

// BLE Event Handlers
void cartConnected(BLEDevice dev) {
  Serial.println("Connected to " + dev.address());
  digitalWrite(ledPin, HIGH);  
}
void cartDisconnected(BLEDevice dev) {
  Serial.println("Disconnected from " + dev.address());
  digitalWrite(ledPin, LOW);  
}
void wheelHandler(BLEDevice dev, BLECharacteristic characteristic) {
  int omega[2];
  wheelRef.readValue(&omega, 8);
  Serial.println(String(omega[0]) + " " + String(omega[1]));
}


// Motor Interrupts from Encoders
void motorAInterrupt() {
  motorA.readEncoder();
}
void motorBInterrupt() {
  motorB.readEncoder();
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
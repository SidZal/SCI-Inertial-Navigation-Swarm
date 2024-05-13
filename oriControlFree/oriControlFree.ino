#include "Motor.h"
#include <ArduinoBLE.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MAX_RPM 255

// IMU Setup
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float quatsypr[7];
float orientation[3];
int16_t motion6[6];

// Motor + Encoder Pins
#define PWMA 6
#define in1 8
#define in2 7
#define C1A 3
#define C2A 5

#define MOTOR_STANDBY 9

#define PWMB 12
#define in3 10
#define in4 11
#define C1B 2
#define C2B 4

#define pi 3.1415926

// Low-Level Speed Control Parameters
const float CPR = 3;
const float gearRatio = 100.37;
const float conversionRatio = 4./CPR*60/gearRatio;
Motor motorA(in1, in2, PWMA, C1A, C2A, conversionRatio);
Motor motorB(in3, in4, PWMB, C1B, C2B, conversionRatio);

float v[2];

// Orientation Controller Variables
unsigned long millisLast = 0;
int wL = 0, wR = 0;
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

  motorA.tunePID(1, 4, 0);
  motorB.tunePID(1, 4, 0);

}

void loop() {
  if(readIMU()) {
    double dt = (millis() - millisLast)/1000.0;
    millisLast = millis();

    float ref = pi*cos(millis()/1000.);
    float e_deg = (ref - quatsypr[4]) * 180/pi;
    integrator += e_deg*dt;
    float e_der = (e_deg-e_prev)/dt;
    e_prev = e_deg;
    int controlIn = 1*e_deg + 3*integrator + .3*e_der;
    Serial.println(String(dt) + ' ' + String(e_deg) + ' ' + String(integrator) + ' ' + String(e_der) + ' ' + String(quatsypr[4]) + ' ' + String(ref) + ' ' + String(3.14) + ' ' + String(-3.14));
    
    motorA.drive(-controlIn);
    motorB.drive(-controlIn);
  }
}

// Motor Interrupts from Encoders
void motorAInterrupt() {
  motorA.readEncoder();
}
void motorBInterrupt() {
  motorB.readEncoder();
}

bool readIMU() {
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
    return true;
  }
  else
    return false;
  
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
#ifndef Motor_h
#define Motor_h
#include "Arduino.h"

class Motor {
public:
  Motor(int in1, int in2, int PWM, int C1, int C2, float encConvRatio); // Constructor

  void setPWM(int newPWM);
  void run(int time);
  void toggleDir();
  void setDir(bool newDir);

  // Allows +/- inputs
  void drive(int target);

  float updateVelocity();
  float rpmControl(float target);
  void readEncoder();
  void tunePID(float kp, float ki, float kd);

private:
  // Pins
  int in1, in2, PWM, C1, C2;

  // Constants
  float encConvRatio;
  int maxRPM;

  // Trackers
  bool dir;

  // Velocity Measurement
  float deltaT;
  long prevT;
  int posPrev;

  // Voltaile Measurements
  volatile int pos_i;
  volatile float velocity_i;
  volatile long prevT_i;

  // for lowPass filter
  float lowPassFilter(float calc, float prevCalc, float prevFilt);
  float v1Filt, v1Prev, v2Filt, v2Prev;

  // PI Controller
  float eintegral, prevE;
  float kp, ki, kd;
};

#endif
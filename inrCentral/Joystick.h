#ifndef Joystick_h
#define Joystick_h
#include "Arduino.h"

class Joystick {
public:
  Joystick(int pin, int max = 100);

  // Run once in setup
  int calibrate(int reps = 5);
  
  // Returns -1 to 1 based on current joystick measurement
  int read();

  // Returns -max to max based on summed joystick measurement. Must be run consistently to keep tally
  float total();

  void setMax(float max);
  void setRate(float rate);
  int getIdle();

private:
  int pin;
  int idle;
  float tally;
  float max;
  float rate;
};

#endif
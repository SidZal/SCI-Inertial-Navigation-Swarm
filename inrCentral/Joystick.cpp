#include "Arduino.h"
#include "Joystick.h"

Joystick::Joystick(int pin, int max) {
  this->pin = pin;
  pinMode(pin, INPUT);

  this->idle = 512;
  this->tally = 0;
  this->max = max;
  this->rate = .05;
}

int Joystick::calibrate(int reps) {
  if(reps < 1)
    return -1;

  int sum = 0;
  for(int i=0; i<reps; i++) {
    sum += analogRead(pin);
    delay(100);
  }

  return idle = sum/reps;
}

int Joystick::readRaw() {
  return analogRead(pin);
}

float Joystick::read() {
  return (analogRead(pin) - idle)/513.;
}

float Joystick::total() {
  float val = rate*this->readRaw();
  if(abs(val) > rate*.1) {
    tally += val;
    if(abs(tally) > max)
      tally = tally/abs(tally) * max;
  }
  
  return tally;
}

void Joystick::setMax(float max) {
  this->max = max;
}

void Joystick::setRate(float rate) {
  this->rate = rate;
}

int Joystick::getIdle() {
  return idle;
}
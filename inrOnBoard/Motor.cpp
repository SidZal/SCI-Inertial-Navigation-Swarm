#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int in1, int in2, int PWM, int C1, int C2, float encConvRatio) {
  // Pins
  this->in1 = in1;
  this->in2 = in2;
  this->PWM = PWM;
  this->C1 = C1;
  this->C2 = C2;

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  // Constants
  this->encConvRatio = encConvRatio;

  // Initial Direction, Power
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  this->dir = true;
  //analogWrite(PWM, 0); // This line ensures cosntant 0 PWM output on nano 33 ble, not sure why

  // Feedback Parameters
  prevT = prevT_i = 0; // long
  posPrev = pos_i = 0; // int
  velocity_i = v1Filt = v1Prev = v2Filt = v2Prev = eintegral = prevE = 0; // float

  kp = 1;
  ki = 3;
  kd = .2;
}

void Motor::setPWM(int newPWM){
  analogWrite(PWM, newPWM);
}

void Motor::run(int time){
  delay(time);
  analogWrite(PWM, 0);
}

void Motor::toggleDir(){
  digitalWrite(in1, dir);
  digitalWrite(in2, !dir);
  dir = !dir;
}

void Motor::setDir(bool newDir) {
  if(dir!=newDir) {
    dir = newDir;
    digitalWrite(in1, !dir);
    digitalWrite(in2, dir); 
  }
}

void Motor::drive(int target) {
  this->setDir(target>0);
  this->setPWM(min(abs(target), 255));
}

float Motor::updateVelocity() {
  // Read position, velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts();
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts();

  // Method 1 Velocity Measurent
  long currT = micros();
  deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert to RPM
  float v1 = velocity1*encConvRatio;
  float v2 = velocity2*encConvRatio;

  // Filter
  v1Filt = lowPassFilter(v1, v1Prev, v1Filt);
  v1Prev = v1;
  v2Filt = lowPassFilter(v2, v2Prev, v2Filt);
  v2Prev = v2;

  return v2Filt;
}

float Motor::rpmControl(float target) {
  float e = target - v2Filt;

  eintegral = eintegral + e*deltaT;
  float ederivative = (e - prevE)/deltaT;
  prevE = e;
  
  return kp*e + ki*eintegral + kd*ederivative;
}

void Motor::tunePID(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

float Motor::lowPassFilter(float calc, float prevCalc, float prevFilt) {
  return 0.854*prevFilt + .0728*calc + .0728*prevCalc;
}

void Motor::readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(C2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
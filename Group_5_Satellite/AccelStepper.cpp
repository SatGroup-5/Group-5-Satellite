// AccelStepper.cpp 

#include "AccelStepper.h"

// Initializes the stepper driver pins and internal state
AccelStepper::AccelStepper(uint8_t interfaceType, uint8_t stepPin, uint8_t dirPin)
  : _interfaceType(interfaceType), _stepPin(stepPin), _dirPin(dirPin),
    _speed(0), _acceleration(0), _maxSpeed(1.0), _currentPos(0), _lastStepTime(0) {
  pinMode(_stepPin, OUTPUT); // set step pin as output
  pinMode(_dirPin, OUTPUT);  // set direction pin as output
}

// Sets the maximum speed allowed (in steps per second)
void AccelStepper::setMaxSpeed(float speed) {
  _maxSpeed = speed;
}

// Sets the acceleration (not used directly in this minimal version, but stored)
void AccelStepper::setAcceleration(float acceleration) {
  _acceleration = acceleration;
}

// Sets current speed and adjusts direction pin based on sign
void AccelStepper::setSpeed(float speed) {
  _speed = constrain(speed, -_maxSpeed, _maxSpeed); // clamp to max speed
  digitalWrite(_dirPin, _speed >= 0 ? HIGH : LOW);  // set motor direction
}

// Runs motor at the set speed by sending step pulses at timed intervals
void AccelStepper::runSpeed() {
  unsigned long time = micros();
  if ((time - _lastStepTime) >= (1000000.0 / abs(_speed))) {
    _lastStepTime = time;
    step(1); // send step pulse
    _currentPos += (_speed > 0) ? 1 : -1; // update position
  }
}

// Sends a single step pulse (HIGH then LOW)
void AccelStepper::step(uint8_t step) {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(_stepPin, LOW);
} 

// AccelStepper.h 

#ifndef AccelStepper_h
#define AccelStepper_h

#include <Arduino.h>

class AccelStepper {
public:
    // Supported interface type: DRIVER mode (step/dir pins)
    static const uint8_t DRIVER = 1;

    // Constructor for DRIVER mode
    AccelStepper(uint8_t interfaceType = DRIVER, uint8_t stepPin = 2, uint8_t dirPin = 3);

    // Set maximum speed (steps per second)
    void setMaxSpeed(float speed);

    // Set acceleration value (optional — not used in minimal loop)
    void setAcceleration(float acceleration);

    // Set the desired speed
    void setSpeed(float speed);

    // Run motor at constant speed — must be called frequently in loop()
    void runSpeed();

protected:
    // Send one step pulse
    void step(uint8_t step);

private:
    uint8_t _interfaceType; // DRIVER mode identifier
    uint8_t _stepPin;       // Arduino pin for step signal
    uint8_t _dirPin;        // Arduino pin for direction signal

    float _speed;           // Current speed in steps/sec
    float _acceleration;    // Desired acceleration (stored)
    float _maxSpeed;        // Limit on speed

    long _currentPos;       // Position tracker (not actively used)
    unsigned long _lastStepTime; // Timestamp of last step pulse
};

#endif

# Satellite Control with Reaction Wheel and Wireless Telemetry

This project demonstrates a model satellite attitude control system. Its primary function is to maintain a stable orientation by using a reaction wheel, controlled by a PID algorithm that responds to data from a gyroscope. The system also integrates environmental sensors and features a wireless communication link for real-time telemetry and remote command execution.

## Table of Contents
1.  [Overview](#overview)
2.  [Core Functionality](#core-functionality)
3.  [Hardware Components](#hardware-components)
4.  [Software and Libraries](#software-and-libraries)
5.  [Operational Logic](#operational-logic)
6.  [Wireless Communication Protocol](#wireless-communication-protocol)
7.  [How to Use](#how-to-use)

## Overview

This project simulates a single-axis satellite stabilisation system. An MPU6050 Inertial Measurement Unit (IMU) detects any angular velocity (rotation) around the Z-axis. This rotation data is fed into a Proportional-Integral-Derivative (PID) controller, which calculates the necessary corrective force. The force is applied by precisely changing the speed of a TMC2209-driven stepper motor, which acts as a reaction wheel. By spinning the wheel in the opposite direction of the external rotation, the system conserves angular momentum and stabilises the main body.

In addition to stabilisation, the system gathers environmental data (pressure, temperature, and calculated altitude) via a BMP280 sensor and features a secondary deployment mechanism simulated by an SG90 servo motor. All data and system status are continuously transmitted wirelessly via an APC220 radio module, and the system can receive commands to alter its operation in real-time.

## Core Functionality

* **Active Stabilisation:** Employs a reaction wheel to actively counteract and nullify angular rotation on a single axis.
* **PID Control:** Utilises a PID control loop for smooth, responsive, and stable adjustments to the reaction wheel's speed.
* **Environmental Sensing:** Measures barometric pressure and temperature to calculate and report the current altitude.
* **Wireless Telemetry:** Transmits all sensor data (angular velocity, motor control signal, altitude, pressure, temperature, servo angle) to a ground station via an APC220 wireless link.
* **Remote Command and Control:** Listens for and executes commands received over the wireless link, including servo movement, motor shutdown, and sea-level pressure calibration.
* **Simulated Deployment:** Features a timed, gradual servo movement to simulate the deployment of a satellite component like an antenna or solar panel.

## Hardware Components

| Component                 | Role                                                                                       | Pin Connections                |
| :------------------------ | :----------------------------------------------------------------------------------------- | :----------------------------- |
| **Arduino/Microcontroller** | Central processing unit running the control logic.                                         | -                              |
| **MPU6050 IMU** | Senses angular velocity and acceleration. Provides the primary input for the stabilisation system. | I2C Bus (SDA, SCL)             |
| **TMC2209 Stepper Driver** | Provides quiet and precise control of the stepper motor.                                   | `DIR: D2`, `STEP: D3`, `EN: D4`  |
| **NEMA Stepper Motor** | Acts as the reaction wheel. By accelerating it, a counter-torque is generated.             | Connected to TMC2209           |
| **BMP280 Sensor** | Measures barometric pressure and temperature.                                              | I2C Bus (SDA, SCL)             |
| **SG90 Servo Motor** | Simulates a deployment mechanism with a timed, gradual movement.                           | `PWM: D9`                      |
| **APC220 Radio Module** | Provides a transparent wireless serial link for telemetry and remote commands.             | `RX: D7`, `TX: D5` (SoftwareSerial) |

## Software and Libraries

The project relies on several key Arduino libraries to interface with the hardware components:

* `<Wire.h>`: For I2C communication with the MPU6050 and BMP280 sensors.
* `<Adafruit_MPU6050.h>`: A library for easy interfacing with the MPU6050 sensor.
* `<AccelStepper.h>`: Enables advanced control of the stepper motor, including acceleration and speed management.
* `<PID.h>`: A custom PID controller library to implement the control algorithm.
* `<Servo.h>`: Standard library for controlling the SG90 servo motor.
* `<i2c_BMP280.h>`: A library for interfacing with the BMP280 sensor via I2C.
* `<SoftwareSerial.h>`: Enables serial communication on digital pins for the APC220 module.

## Operational Logic

1.  **Setup Phase:**
    * Initialises serial communication for both the wired monitor and the wireless APC220.
    * Enables the TMC2209 stepper driver.
    * Initialises the MPU6050 and BMP280 sensors on the I2C bus.
    * Performs a **gyro calibration** by taking 500 readings while stationary to determine the inherent sensor bias (offset). Important for accurate readings.
    * Sets the initial parameters for the stepper motor and moves the servo to its starting position (180 degrees).

2.  **Main Loop:**
    * **Read Gyro:** Measures the current angular velocity around the Z-axis from the MPU6050 and subtracts the calibration offset. A deadzone is applied to ignore minor sensor noise.
    * **Listen for Commands:** Checks the APC220 for incoming wireless commands and executes them if received.
    * **PID Calculation:** The PID controller compares the current `angularVelocity` to a `setpoint` of 0.0 (no rotation). It calculates a `controlSignal` representing the required motor speed to counteract the rotation.
    * **Control Reaction Wheel:** The `controlSignal` is sent to the stepper motor. A negative sign ensures the motor spins in the opposite direction of the detected rotation.
    * **Servo Deployment:** After a 10-second delay from startup, the servo motor begins a slow, controlled movement from 180 to 90 degrees.
    * **Read Environmental Data:** Reads the pressure and temperature from the BMP280 and calculates the altitude based on the current pressure and the stored sea-level pressure.
    * **Transmit Telemetry:** All key data points are formatted and sent out over both the wired serial monitor and the wireless APC220 link.

## Wireless Communication Protocol

The system uses a simple, text-based command protocol over the wireless serial link. All commands must be terminated with a newline character (`\n`).

* **`SERVO_180`**: Resets the servo's target position to 180 degrees.
* **`SERVO_90`**: Sets the servo's target position to 90 degrees.
* **`STOP_MOTOR`**: Immediately stops the reaction wheel motor.
* **`SEALEVEL_[pressure]`**: Updates the sea-level pressure value used for altitude calculations.
    * Example: `SEALEVEL_1013.25` updates the value to 1013.25 hPa.

## How to Use

1.  **Assemble the Hardware:** Connect all components according to the pinout defined in the code.
2.  **Upload the Code:** Compile and upload the `Group_5_Satellite.ino` sketch to your Arduino board.
3.  **Calibrate:** Upon startup, keep the device perfectly still for a few seconds while the gyro calibration runs. The Serial Monitor will indicate when it is complete.
4.  **Monitor:** Open the Arduino Serial Monitor at **115200 baud** to view the live telemetry stream.
5.  **Wireless Link:** Power a second Arduino connected to another APC220 module and a computer. Use its serial monitor (at **9600 baud**) to receive the wireless telemetry and send commands.
6.  **Test:** Gently rotate the assembly around its Z-axis. You should observe the reaction wheel spinning up to counteract the motion and see the "Angular Velocity" in the telemetry approach zero.

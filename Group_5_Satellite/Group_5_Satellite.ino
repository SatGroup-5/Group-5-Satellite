// Group_5_Satellite.ino
// Reaction wheel system using MPU6050 gyro, TMC2209 stepper motor, SG90 servo,
// PID control, BMP280 pressure sensor, and APC220 for wireless serial communication.

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "AccelStepper.h"
#include "PID.h"
#include <Servo.h>
#include "i2c.h"
#include "i2c_BMP280.h"
#include <SoftwareSerial.h>
SoftwareSerial apc(5, 7); // Create software serial for APC220 (TX on D5, RX on D7)

// --- PIN SETUP ---
#define STEP_PIN 3         // Step signal for stepper driver
#define DIR_PIN 2          // Direction signal for stepper driver
#define EN_PIN 4           // Enable signal for stepper driver (LOW = enabled)
#define SERVO_PIN 9        // PWM pin connected to SG90 servo

// --- OBJECTS ---
Adafruit_MPU6050 mpu;                             // Gyroscope & accelerometer
AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  // Stepper motor object

// --- PID controller ---(P, I, D) Proportional gain (how strongly the controller responds to the current error)-Integral gain (continuously adjusting the control signal)-Derivative gain (helps dampen oscillations and improve the stability)
PIDController pid(1.0, 0.2, 0.1);                 // PID controller (P, I, D) 
Servo myServo;                                    // Servo motor
BMP280 bmp280;                                    // BMP280 altitude/pressure/temp sensor

// --- TIMING & STATE ---
unsigned long lastTime = 0;
unsigned long servoStartTime;         // Time when servo movement begins
bool servoMoved = false;              // Flag to avoid repeating the servo movement
float gyroZOffset = 0;                // Offset for Z-axis gyro (for calibration)

int currentServoPos = 180;            // Current angle of the servo
int targetServoPos = 90;              // Target angle to move toward
unsigned long lastServoMoveTime = 0;  // Last time servo moved a degree
const int servoStepDelay = 30;        // Delay between servo steps in ms

float seaLevel_hPa = 1020.0;          // Sea-level pressure (can be updated via APC)

// --- FUNCTIONS ---

// Gyro Z-axis calibration: takes 500 samples to find static bias
void calibrateGyroZ() {
  Serial.println("Calibrating gyro Z... Keep the device still");
  float sum = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(2);
  }
  gyroZOffset = sum / samples;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset, 6);
}

void setup() {
  Serial.begin(115200);
  apc.begin(9600);              // Start APC220 serial
  delay(100);

  pinMode(EN_PIN, OUTPUT);      // Enable pin for stepper driver
  digitalWrite(EN_PIN, LOW);    // LOW = enable TMC2209 driver

  // --- Initialize MPU6050 ---
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  calibrateGyroZ();

  // --- Initialize stepper motor parameters ---
  motor.setMaxSpeed(1000);
  motor.setAcceleration(500);

  // --- Initialize servo ---
  myServo.attach(SERVO_PIN);
  myServo.write(180);             // Start at 180 degrees
  servoStartTime = millis();      // Record when servo was initialized

  // --- Initialize BMP280 sensor ---
  Serial.print("Probe BMP280: ");
  if (bmp280.initialize()) Serial.println("Sensor found");
  else {
    Serial.println("Sensor missing");
    while (1) {}
  }
  bmp280.setEnabled(0);
  bmp280.triggerMeasurement();
}

void loop() {
  // --- Read sensor data from MPU6050 ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angularVelocity = (g.gyro.z - gyroZOffset) * 180.0 / PI; // Convert from RAD/s to deg/s
  if (abs(angularVelocity) < 0.3) angularVelocity = 0; // Deadzone for small noise

  float setpoint = 0.0;
  unsigned long now = millis();
  long deltaT = now - lastTime;
  lastTime = now;

  // --- Check for incoming commands from APC220 ---
  if (apc.available()) {
    String command = apc.readStringUntil('\n');
    command.trim();  // Remove whitespace and newline

    if (command == "SERVO_180") {
      targetServoPos = 180;
      servoMoved = false;
      Serial.println("Received command: SERVO_180");
    } 
    else if (command == "SERVO_90") {
      targetServoPos = 90;
      servoMoved = false;
      Serial.println("Received command: SERVO_90");
    } 
    else if (command == "STOP_MOTOR") {
      motor.setSpeed(0);
      Serial.println("Received command: STOP_MOTOR");
    }
    else if (command.startsWith("SEALEVEL_")) {
      String valueStr = command.substring(9);
      float newPressure = valueStr.toFloat();
      if (newPressure > 800 && newPressure < 1100) {
        seaLevel_hPa = newPressure;
        Serial.print("Updated sea-level pressure to ");
        Serial.println(seaLevel_hPa);
        apc.print("Updated sea-level pressure to ");
        apc.println(seaLevel_hPa);
      } else {
        Serial.println("Invalid sea-level pressure value");
      }
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }

  // --- PID motor control ---
  float controlSignal = pid.compute(setpoint, angularVelocity, deltaT);
  controlSignal = constrain(controlSignal, -1000, 1000); // Steps/sec = 1000 ÷ 200 = 5 revolutions per second = 300 RPM.
  if (angularVelocity == 0.0) {
    pid.reset();
    controlSignal = 0;
  }

  motor.setSpeed(-controlSignal);
  motor.runSpeed();

  // --- Gradual servo movement after 10 seconds ---
  if (!servoMoved && (now - servoStartTime >= 10000)) {
    if (millis() - lastServoMoveTime >= servoStepDelay && currentServoPos > targetServoPos) {
      currentServoPos--;
      myServo.write(currentServoPos);
      lastServoMoveTime = millis();
    }
    if (currentServoPos == targetServoPos) {
      servoMoved = true;
    }
  }

  // --- BMP280 Sensor Readings ---
  bmp280.awaitMeasurement();
  float temperature, pascal, meters;
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  meters = 44330.0 * (1.0 - pow(pascal / (seaLevel_hPa * 100), 0.1903)); // Altitude calculation
  bmp280.triggerMeasurement();

  // --- Print sensor and system data to Serial Monitor ---
  Serial.print("Angular Velocity: ");
  Serial.print(angularVelocity);
  Serial.print(" deg/s\tControl: ");
  Serial.print(controlSignal);
  Serial.print("\tAltitude: ");
  Serial.print(meters);
  Serial.print(" m\tPressure: ");
  Serial.print(pascal);
  Serial.print(" Pa\tTemp: ");
  Serial.print(temperature);
  Serial.print(" C\tServo Angle: ");
  Serial.println(currentServoPos);

  // --- Print same data to APC220 ---
  
  apc.print("Angular Velocity: ");
  apc.print(angularVelocity);
  apc.print(" deg/s\tControl: ");
  apc.print(controlSignal);
  apc.print("\tAltitude: ");
  apc.print(meters);
  apc.print(" m\tPressure: ");
  apc.print(pascal);
  apc.print(" Pa\tTemp: ");
  apc.print(temperature);
  apc.print(" C\tServo Angle: ");
  apc.println(currentServoPos);
  
  delay(5);  // Small delay to avoid flooding the output
}

#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Pin assignment for BFD-1000 sensor array
const uint8_t sensorPins[SensorCount] = {A0, A1, A2, A3, A4};

// L298N Motor Controller Pins
const int IN1 = 8;   // Left motor IN1
const int IN2 = 9;   // Left motor IN2
const int ENA = 7;   // Left motor PWM (ENA)
const int IN3 = 10;    // Right motor IN3
const int IN4 = 11;    // Right motor IN4 
const int ENB = 12;    // Right motor PWM (ENB)

// PID constants
float Kp = 3 ;  // Proportional constant
float Kd = 4;  // Derivative constant
float Ki = 0.0002; // Integral constant

// Base speed for motors (PWM range is 0 to 255)
const int baseSpeed = 160;

// Variables for PID
int16_t last_error = 0;
int16_t integral = 0;

// New variables for smoothing
const int numReadings = 5;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

void setup() {
  Serial.begin(115200);
  
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);
  
  Serial.println("Calibrating. Please move the sensor over the line...");
  
  for (uint16_t i = 0; i < 500; i++) {
    qtr.calibrate();
    delay(1);
  }
  
  Serial.println("Calibration finished");

  // Initialize all the readings to 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

int isSharpTurn() {
  if (sensorValues[0] > 900 && sensorValues[1] > 900) {
    return -1;  // Sharp left turn
  } else if (sensorValues[3] > 900 && sensorValues[4] > 900) {
    return 1;   // Sharp right turn
  }
  return 0;     // No sharp turn
}

int smoothError(int error) {
  total = total - readings[readIndex];
  readings[readIndex] = error;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;

  return total / numReadings;
}

void loop() {
  // Line following algorithm
  uint16_t position = qtr.readLineBlack(sensorValues);
  int16_t error = 2000 - position;
  
  // Apply smoothing to the error
  error = smoothError(error);
  
  Serial.println(error);
  
  // Check if robot has left the line
  bool off_line = true;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 900) {  // Assuming 900 is the threshold for detecting the line
      off_line = false;
      break;
    }
  }

  if (off_line) {
    // Robot has left the line, perform recovery
    if (last_error > 0) {
      // Turn left if the line was to the left before
      setMotorSpeeds(-baseSpeed, baseSpeed);
    } else {
      // Turn right if the line was to the right before
      setMotorSpeeds(baseSpeed, -baseSpeed);
    }
  } else {
    // PID control
    integral += error;
    int16_t derivative = error - last_error;
    
    int motorSpeed = Kp * error + Ki * integral + Kd * derivative;
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    
    int sharpTurn = isSharpTurn();
    if (sharpTurn != 0) {
      // More gentle turn strategy for sharp turns
      if (sharpTurn > 0) {  // Sharp right turn
        leftSpeed = baseSpeed + 50;
        rightSpeed = baseSpeed - 50;
      } else {  // Sharp left turn
        leftSpeed = baseSpeed - 50;
        rightSpeed = baseSpeed + 50;
      }
    } else {
      // Normal PID control
      leftSpeed -= motorSpeed;
      rightSpeed += motorSpeed;
    }
    
    // Motor speed adjustment
    leftSpeed = constrain(leftSpeed, -100, 255);
    rightSpeed = constrain(rightSpeed, -100, 255);
    
    // Control motors based on calculated speeds
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // Update last_error for next iteration
    last_error = error;
    
    // Prevent integral windup
    integral = constrain(integral, -1000, 1000);
  }

  delay(10);  // Small delay to reduce update frequency and smoothen movement
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Control left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;  // Make speed positive for PWM
  }
  analogWrite(ENA, leftSpeed);

  // Control right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;  // Make speed positive for PWM
  }
  analogWrite(ENB, rightSpeed);
}
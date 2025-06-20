/*
 * Arduino Robot Control for Stop Sign Detection System
 * 
 * This code receives commands from Python via UART:
 * - '1' = Stop the robot
 * - '0' = Keep moving
 * 
 * The robot uses two DC motors controlled by a motor driver
 * (L298N or similar) connected to the Arduino pins.
 */

// Motor control pins (adjust based on your motor driver)
const int motorLeftForward = 2;   // Left motor forward pin
const int motorLeftBackward = 3;  // Left motor backward pin
const int motorRightForward = 4;  // Right motor forward pin
const int motorRightBackward = 5; // Right motor backward pin

// Motor speed control pins (PWM pins for speed control)
const int motorLeftSpeed = 9;     // Left motor speed (PWM)
const int motorRightSpeed = 10;   // Right motor speed (PWM)

// Status LED pin
const int statusLED = 13;         // Built-in LED for status indication

// Robot control variables
int robotState = 0;               // 0 = moving, 1 = stopped
int motorSpeed = 150;             // Default motor speed (0-255)
unsigned long lastCommandTime = 0; // For timeout detection
const unsigned long commandTimeout = 1000; // 1 second timeout

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize motor control pins as outputs
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  pinMode(motorLeftSpeed, OUTPUT);
  pinMode(motorRightSpeed, OUTPUT);
  
  // Initialize status LED
  pinMode(statusLED, OUTPUT);
  
  // Start with robot stopped for safety
  stopRobot();
  
  Serial.println("Arduino Robot Control System Initialized");
  Serial.println("Waiting for commands from Python...");
  
  // Flash LED to indicate system ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(statusLED, HIGH);
    delay(200);
    digitalWrite(statusLED, LOW);
    delay(200);
  }
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char command = Serial.read();
    lastCommandTime = millis();
    
    if (command == '1') {
      // Stop command received
      if (robotState != 1) {
        robotState = 1;
        stopRobot();
        digitalWrite(statusLED, HIGH); // LED on when stopped
        Serial.println("STOP command received - Robot stopped");
      }
    }
    else if (command == '0') {
      // Move command received
      if (robotState != 0) {
        robotState = 0;
        moveForward();
        digitalWrite(statusLED, LOW); // LED off when moving
        Serial.println("MOVE command received - Robot moving");
      }
    }
  }
  
  // Safety timeout - stop robot if no command received for too long
  if (millis() - lastCommandTime > commandTimeout && robotState == 0) {
    robotState = 1;
    stopRobot();
    digitalWrite(statusLED, HIGH);
    Serial.println("Communication timeout - Robot stopped for safety");
  }
  
  // Small delay to prevent overwhelming the serial buffer
  delay(10);
}

void moveForward() {
  /*
   * Move robot forward by activating both motors in forward direction
   */
  // Left motor forward
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  analogWrite(motorLeftSpeed, motorSpeed);
  
  // Right motor forward
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
  analogWrite(motorRightSpeed, motorSpeed);
}

void stopRobot() {
  /*
   * Stop all robot movement by turning off all motors
   */
  // Stop left motor
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, LOW);
  analogWrite(motorLeftSpeed, 0);
  
  // Stop right motor
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, LOW);
  analogWrite(motorRightSpeed, 0);
}

void moveBackward() {
  /*
   * Move robot backward (optional function for future enhancements)
   */
  // Left motor backward
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, HIGH);
  analogWrite(motorLeftSpeed, motorSpeed);
  
  // Right motor backward
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, HIGH);
  analogWrite(motorRightSpeed, motorSpeed);
}

void turnLeft() {
  /*
   * Turn robot left (optional function for future enhancements)
   */
  // Left motor backward, right motor forward
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, HIGH);
  analogWrite(motorLeftSpeed, motorSpeed);
  
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
  analogWrite(motorRightSpeed, motorSpeed);
}

void turnRight() {
  /*
   * Turn robot right (optional function for future enhancements)
   */
  // Left motor forward, right motor backward
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  analogWrite(motorLeftSpeed, motorSpeed);
  
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, HIGH);
  analogWrite(motorRightSpeed, motorSpeed);
}

/*
 * Pin Connection Guide:
 * 
 * L298N Motor Driver to Arduino:
 * - IN1 → Pin 2 (motorLeftForward)
 * - IN2 → Pin 3 (motorLeftBackward)
 * - IN3 → Pin 4 (motorRightForward)
 * - IN4 → Pin 5 (motorRightBackward)
 * - ENA → Pin 9 (motorLeftSpeed)
 * - ENB → Pin 10 (motorRightSpeed)
 * - VCC → 5V
 * - GND → GND
 * 
 * Motors to L298N:
 * - Left Motor → OUT1, OUT2
 * - Right Motor → OUT3, OUT4
 * 
 * Power Supply:
 * - Connect external power supply (6-12V) to L298N VCC and GND
 * - Remove VCC jumper on L298N if using external power
 * 
 * Status LED:
 * - Built-in LED on Pin 13 (no external connection needed)
 * - Optional: Connect external LED with resistor to Pin 13
 */
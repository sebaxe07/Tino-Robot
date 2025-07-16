#include <CytronMotorDriver.h>
#include "pinout.h"

// Simple debug flag
#define DEBUG_MODE true

// Motor speed settings
#define BASE_SPEED 150      // Base PWM speed (0-255)
#define TURN_SPEED 80       // Turning speed

// Configure the motor drivers using PWM_DIR mode
CytronMD rightMotor(PWM_DIR, _1_1A, _1_1B);   // Right motor
CytronMD leftMotor(PWM_DIR, _1_2A, _1_2B);    // Left motor

// Variables for command processing
char incomingChar = 0;
bool motorActive = false;
char lastCommand = 0;
unsigned long lastCommandTime = 0;
unsigned long commandTimeout = 200;  // Stop motors if no command received for 200ms

// Debug print helper
void debug_print(const char* message) {
  if (DEBUG_MODE) {
    Serial.print(message);
  }
}

void debug_println(const char* message) {
  if (DEBUG_MODE) {
    Serial.println(message);
  }
}

// Movement functions
void moveForward() {
  leftMotor.setSpeed(BASE_SPEED);
  rightMotor.setSpeed(-BASE_SPEED);  // Inverted for correct direction
  debug_println("Moving FORWARD");
  motorActive = true;
}

void moveBackward() {
  leftMotor.setSpeed(-BASE_SPEED);
  rightMotor.setSpeed(BASE_SPEED);   // Inverted for correct direction
  debug_println("Moving BACKWARD");
  motorActive = true;
}

void turnLeft() {
  leftMotor.setSpeed(-TURN_SPEED);
  rightMotor.setSpeed(-TURN_SPEED);
  debug_println("Turning LEFT");
  motorActive = true;
}

void turnRight() {
  leftMotor.setSpeed(TURN_SPEED);
  rightMotor.setSpeed(TURN_SPEED);
  debug_println("Turning RIGHT");
  motorActive = true;
}

void stopMotors() {
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  if (motorActive) {
    debug_println("STOPPED");
    motorActive = false;
  }
}

void processCommand(char command) {
  lastCommand = command;
  lastCommandTime = millis();
  
  switch (command) {
    case 'w':
    case 'W':
      moveForward();
      break;
      
    case 's':
    case 'S':
      moveBackward();
      break;
      
    case 'a':
    case 'A':
      turnLeft();
      break;
      
    case 'd':
    case 'D':
      turnRight();
      break;
      
    case ' ':  // Space bar to stop
    case 'x':
    case 'X':
      stopMotors();
      lastCommand = 0;  // Clear last command
      break;
      
    default:
      // Unknown command - stop motors for safety
      stopMotors();
      lastCommand = 0;  // Clear last command
      debug_print("Unknown command: ");
      debug_println(&command);
      break;
  }
}

void checkCommandTimeout() {
  // Auto-stop if no command received within timeout period
  if (lastCommand != 0 && lastCommand != 'x' && lastCommand != 'X' && lastCommand != ' ') {
    if (millis() - lastCommandTime > commandTimeout) {
      stopMotors();
      lastCommand = 0;
      debug_println("Command timeout - STOPPED");
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(500);
  
  Serial.println("=====DEBUG BASE WASD CONTROLLER=====");
  Serial.println("Commands (HOLD key, don't press Enter):");
  Serial.println("  W - Move Forward");
  Serial.println("  S - Move Backward"); 
  Serial.println("  A - Turn Left");
  Serial.println("  D - Turn Right");
  Serial.println("  X or SPACE - Stop");
  Serial.println("");
  Serial.println("NOTE: Use a terminal program that sends");
  Serial.println("characters immediately (like 'screen' or 'minicom')");
  Serial.println("Arduino Serial Monitor requires Enter.");
  Serial.println("=====================================");
  Serial.println("Ready for commands...");
  
  // Initialize motors to stopped state
  stopMotors();
  lastCommandTime = millis();
}

void loop() {
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    incomingChar = Serial.read();
    
    // Echo the received character for feedback
    Serial.print("Received: ");
    Serial.println(incomingChar);
    
    // Process the command
    processCommand(incomingChar);
  }
  
  // Check if we should auto-stop due to command timeout
  checkCommandTimeout();
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

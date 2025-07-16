#include <CytronMotorDriver.h>
#include <Encoder.h>
#include "RaspberryCommunication.h"
#include "pinout.h"
#include <avr/wdt.h> // Include watchdog timer

// Forward declarations
void process_special_command(char* buffer);
void send_status_update();

// Debug flag - set to true to enable debug prints, false to disable
#define DEBUG_MODE true

// Debug print helper functions
void debug_print(const char* message) {
  if (DEBUG_MODE) {
    Serial.print(message);
  }
}

void debug_print(float value) {
  if (DEBUG_MODE) {
    Serial.print(value);
  }
}

void debug_print(int value) {
  if (DEBUG_MODE) {
    Serial.print(value);
  }
}

void debug_print(unsigned long value) {
  if (DEBUG_MODE) {
    Serial.print(value);
  }
}

void debug_println(const char* message) {
  if (DEBUG_MODE) {
    Serial.println(message);
  }
}

void debug_println(float value) {
  if (DEBUG_MODE) {
    Serial.println(value);
  }
}

void debug_println(int value) {
  if (DEBUG_MODE) {
    Serial.println(value);
  }
}

void debug_println(unsigned long value) {
  if (DEBUG_MODE) {
    Serial.println(value);
  }
}

//_________________CYTRON_MOTOR___________________________//
//Maximum speed wanted
#define _MAX_SPEED 50       //cm/s
#define _MAX_ANGULAR 1.1    // rad/s
#define wheel_radius 3.0f   //cm
#define robot_radius 16.5f  //cm
#define encoder_ppr 1920.0f
//zigler-nichols PID Tuning
#define Ku 21
#define Tu 2.25
#define LOOPTIME 25         // PID loop time in milliseconds

// Define watchdog timeout
#define MAX_WATCHDOG_ELAPSED_TIME 100000
#define DEBUG true // Disable debug output for production

// PID constants
float Kp = 7.3;
float Ki = 5.6;
float Kd = 0.2;

// Buffer for serial reading (using external buffer from RaspberryCommunication.h)
extern char buffer[BUFFER_SIZE]; // Reference the external buffer

// FIXED: Completely swapped left and right motor configurations
// Configure the motor drivers using PWM_DIR mode with correct pin assignments
CytronMD rightMotor(PWM_DIR, _1_1A, _1_1B);   // Now defined as right motor (was left)
CytronMD leftMotor(PWM_DIR, _1_2A, _1_2B);    // Now defined as left motor (was right)

// FIXED: Also swap the encoders
// Set up the encoders with corrected pin assignments
Encoder rightEncoder(_EP31, _EP32);   // Now defined as right encoder (was left)
Encoder leftEncoder(_EP21, _EP22);    // Now defined as left encoder (was right)

// Variables for tracking encoder readings
long leftPosition = 0;
long rightPosition = 0;
long prevLeftPosition = 0;
long prevRightPosition = 0;

// Variables for PID control, similar to ViRHas implementation
double speed_req[2];       // Target speeds
double speed_act[2];       // Actual speeds
int PWM_val[2];            // PWM values to apply
double last_error[2];      // Last error for PID
double Iterm[2];           // Integral term for PID
double lastSP[2];          // Last setpoint
unsigned long lastMillis[2];  // Last time encoder was read
unsigned long lastMilliLoop;  // Last time PID was calculated

unsigned long encoderUpdateTime = 0;
const int encoderUpdateInterval = 500; // Update encoder readings every 500ms

char feedback_msg[100];

// Variables for periodic status updates
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 1000; // Send status every 1 second

//_______________________________
unsigned long phaseStartTimeLinear = 0;  // Timer for linear movement
unsigned long phaseStartTimeAngular = 0; // Timer for angular movement
bool wasLinearYActive = false;     // Tracks if linearY was previously active
bool wasAngularActive = false;     // Tracks if angular was previously active
bool angularDirectionPositive = false; // Track the direction of angular movement

// State tracking for atomic movements that must complete
bool isCase1InProgress = false;    // Tracks if case 1 "little push" is in progress
bool isCase2InProgress = false;    // Tracks if case 2 timing cycle is in progress
bool isCase3InProgress = false;    // Tracks if case 3 movement sequence is in progress
bool isCase2Locked = false;        // Lock system for case 2 - ignores all commands except case 3
bool pendingCase3 = false;         // Tracks if case 3 is waiting for case 2 to complete
float pendingAngular = 0.0;        // Stores angular value for pending case 3 execution
int lastLinearCommand = 0;         // Track the last linearY command received

// Timing constants
const unsigned long CASE2_CYCLE_TIME = 1500;  // 1.5 seconds for case 2 timing cycle
//_______________________________

// Note: last_command_time and current_time are now used from RaspberryCommunication.cpp


// Get motor speed in rad/s from encoder readings
void getMotorSpeed(long deltaT, int pos, int prev_pos, int i) {
  // (Delta Pos * Distance per 1 pulse) / Delta Time
  speed_act[i] = ((pos - prev_pos) * ((2.0f*(float)M_PI)/encoder_ppr)) / (deltaT/1000.0f);
}

// PID controller for motor speed, similar to ViRHas implementation
int updatePid(double targetValue, double currentValue, int i) {
  double pidTerm = 0;
  double error = 0;
  
  if (lastSP[i] != targetValue) {
    Iterm[i] = 0;
  }
  
  error = targetValue - currentValue;
  Iterm[i] += error * Ki;
  double deltaError = last_error[i] - error;
  pidTerm = (Kp * error) + (Iterm[i]) + (Kd * deltaError);
  last_error[i] = error;
  lastSP[i] = targetValue;
  
  return constrain(int(pidTerm), -255, 255);
}

// Calculate motor speeds for differential drive based on linearY and angular
void calculateMotorSpeeds(float linearY, float angular) {
  // Simplified approach: always use maximum speeds when movement is commanded
  
  // For linear movement, use the maximum speed if any command is present
  float actualLinearY = 0;
  if (linearY != 0) {
    // Use maximum speed with appropriate direction
    actualLinearY = linearY > 0 ? _MAX_SPEED : -_MAX_SPEED;
  }
  
  // For angular movement, use the maximum angular speed if any command is present
  float actualAngular = 0;
  if (angular != 0) {
    // Use maximum angular speed with appropriate direction
    actualAngular = angular > 0 ? _MAX_ANGULAR : -_MAX_ANGULAR;
  }
  
  // Now convert to wheel speeds in rad/s
  // For differential drive:
  // INVERTED TURNING: Changed signs of angular component
  // Left wheel speed = (linearY + angular * robot_radius) / wheel_radius
  // Right wheel speed = (linearY - angular * robot_radius) / wheel_radius
  
  // Note: We invert as needed based on motor orientation
  speed_req[0] = (actualLinearY + actualAngular * robot_radius) / wheel_radius;  // Left wheel (angular sign inverted)
  speed_req[1] = -(actualLinearY - actualAngular * robot_radius) / wheel_radius; // Right wheel (both signs inverted)
}

// Run PID loop to control motor speeds
void pidLoop() {
  unsigned long currentMillis = millis();
  unsigned long deltaPid = currentMillis - lastMilliLoop;
  
  if(deltaPid >= LOOPTIME) {
    lastMilliLoop = currentMillis;
    
    // Get current encoder positions
    long leftPos = leftEncoder.read();
    long rightPos = rightEncoder.read();
    
    // Calculate actual speeds
    long deltaT = currentMillis - lastMillis[0];
    getMotorSpeed(deltaT, leftPos, prevLeftPosition, 0);
    lastMillis[0] = currentMillis;
    prevLeftPosition = leftPos;
    
    deltaT = currentMillis - lastMillis[1];
    getMotorSpeed(deltaT, rightPos, prevRightPosition, 1);
    lastMillis[1] = currentMillis;
    prevRightPosition = rightPos;
    
    // Update motor PWM values based on PID
    PWM_val[0] = updatePid(speed_req[0], speed_act[0], 0);
    PWM_val[1] = updatePid(speed_req[1], speed_act[1], 1);
    
    // Apply PWM values
    leftMotor.setSpeed(PWM_val[0]);
    rightMotor.setSpeed(PWM_val[1]);
  }
}

// Adapted moveRobot function for two-wheeled differential drive with PID control
void moveRobot(float linearY, float angular) {
  if (linearY == 0.00 && angular == 0.00) {
    // Stop both motors
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    speed_req[0] = 0;
    speed_req[1] = 0;
  } else {
    // Calculate required wheel speeds
    calculateMotorSpeeds(linearY, angular);
  }
}

void updateBaseMovementByTime(float linearY, float angular) {
  unsigned long currentTime = millis();
  
  // Convert linearY to integer to check for specific command values
  int command = (int)linearY;
  
  // Debug: Print command received (only when it changes)
  if (DEBUG_MODE && command != lastLinearCommand) {
    debug_print("BASE Command: ");
    debug_print(lastLinearCommand);
    debug_print(" -> ");
    debug_print(command);
    debug_print(" | Current time: ");
    debug_println(currentTime);
  }
  
  // Check if case 1 is in progress and must complete
  if (isCase1InProgress) {
    unsigned long elapsedTimeLinear = currentTime - phaseStartTimeLinear;
    // Little push movement profile for command 1 - must complete
    if (elapsedTimeLinear < 600) {
      linearY = 0;  // First phase: 600ms delay before movement
    } else if (elapsedTimeLinear >= 600 && elapsedTimeLinear < 800) {
      linearY = 12;  // Second phase: 200ms forward movement at moderate speed
    } else if (elapsedTimeLinear >= 800 && elapsedTimeLinear < 1000) {
      linearY = -12;  // Third phase: 200ms backward movement to return to original position
    } else {
      linearY = 0;  // Stop movement after completing the push cycle
      wasLinearYActive = false;  // Reset the state
      isCase1InProgress = false;  // Mark case 1 as completed
      debug_println("BASE Case 1 completed");
    }
    lastLinearCommand = command;
    moveRobot(linearY, angular);
    return; // Exit early to prevent other command processing
  }
  
  // Check if case 2 is in progress and must complete (blocks ALL commands including case 3)
  if (isCase2InProgress) {
    unsigned long elapsedTimeLinear = currentTime - phaseStartTimeLinear;
    // Case 2: Timing cycle that matches leg timing - no movement, just waiting
    debug_print("BASE Case 2 timing: elapsed=");
    debug_print(elapsedTimeLinear);
    debug_print("ms, target=");
    debug_print(CASE2_CYCLE_TIME);
    debug_print("ms, remaining=");
    debug_println(CASE2_CYCLE_TIME - elapsedTimeLinear);
    
    if (elapsedTimeLinear < CASE2_CYCLE_TIME) {  // Use variable for timing cycle
      debug_println("BASE Case 2 timing cycle in progress - blocking all commands");
      
      // Check if case 3 command is received during case 2 timing - save it for later
      if (command == 3 && !pendingCase3) {
        pendingCase3 = true;
        pendingAngular = angular;  // Store the angular value for later execution
        debug_print("BASE Case 3 received during case 2 - saving for later execution (angular: ");
        debug_print(pendingAngular);
        debug_println(")");
      }
      
      linearY = 0;  // No movement during timing cycle
    } else {
      linearY = 0;  // Still no movement
      wasLinearYActive = false;  // Reset the state
      isCase2InProgress = false;  // Mark case 2 timing cycle as completed
      isCase2Locked = true;  // NOW lock the system - only case 3 can unlock
      debug_print("BASE Case 2 timing completed after ");
      debug_print(elapsedTimeLinear);
      debug_println("ms - LOCKED for case 3");
      
      // If case 3 was pending, automatically execute it now
      if (pendingCase3) {
        debug_print("BASE Executing pending case 3 after case 2 completion (angular: ");
        debug_print(pendingAngular);
        debug_println(")");
        pendingCase3 = false;  // Clear the pending flag
        isCase2Locked = false;  // UNLOCK the system
        
        // Check what type of pending case 3 based on stored angular value
        if (pendingAngular == 0) {
          // Pending forward movement
          phaseStartTimeLinear = millis();  // Reset timer for case 3
          wasLinearYActive = true;
          wasAngularActive = false;
          isCase3InProgress = true;  // Mark case 3 as started
          debug_print("BASE Auto case 3 forward timer started at: ");
          debug_println(phaseStartTimeLinear);
          linearY = 16;  // Forward movement for ~1.7 seconds
          debug_println("BASE Auto case 3 forward movement started");
        } else {
          // Pending rotation movement
          phaseStartTimeAngular = millis();  // Reset timer for case 3
          wasAngularActive = true;
          wasLinearYActive = false;
          isCase3InProgress = true;  // Mark case 3 as started
          angularDirectionPositive = pendingAngular > 0; // Store direction
          debug_print("BASE Auto case 3 rotation timer started at: ");
          debug_println(phaseStartTimeAngular);
          angular = angularDirectionPositive ? -1.1 : 1.1;  // Start rotation
          debug_print("BASE Auto case 3 rotation ");
          debug_print(angularDirectionPositive ? "RIGHT" : "LEFT");
          debug_println(" movement started");
        }
        pendingAngular = 0.0;  // Clear the stored angular value
      }
    }
    lastLinearCommand = command;
    moveRobot(linearY, 0);  // No angular movement during case 2 progress
    return; // Exit early to prevent other command processing (including case 3)
  }
  
  // Check if case 3 is in progress and must complete
  if (isCase3InProgress) {
    // Check if this is a linear (forward) or angular (rotation) case 3
    if (wasLinearYActive) {
      // Linear case 3 (forward movement)
      unsigned long elapsedTimeLinear = currentTime - phaseStartTimeLinear;
      if (elapsedTimeLinear < 1720) {
        linearY = 16;  // Forward movement for ~1.7 seconds
      } else {
        linearY = 0;  // Stop movement after completing the forward cycle
        wasLinearYActive = false;  // Reset the state
        isCase3InProgress = false;  // Mark case 3 as completed
        debug_println("BASE Case 3 forward completed");
      }
      lastLinearCommand = command;
      moveRobot(linearY, 0);  // No angular movement for forward case
    } else if (wasAngularActive) {
      // Angular case 3 (rotation movement)
      unsigned long elapsedTimeAngular = currentTime - phaseStartTimeAngular;
      if (elapsedTimeAngular < 1720) {
        angular = angularDirectionPositive ? -1.1 : 1.1;  // Continue rotation
        debug_print("BASE Case 3 rotation ");
        debug_print(angularDirectionPositive ? "RIGHT" : "LEFT");
        debug_print(" in progress, elapsed: ");
        debug_println(elapsedTimeAngular);
      } else {
        angular = 0;  // Stop rotation after completing the cycle
        wasAngularActive = false;  // Reset the state
        isCase3InProgress = false;  // Mark case 3 as completed
        debug_print("BASE Case 3 rotation ");
        debug_print(angularDirectionPositive ? "RIGHT" : "LEFT");
        debug_println(" completed");
      }
      lastLinearCommand = command;
      moveRobot(0, angular);  // No linear movement for rotation case
    } else {
      // Fallback - shouldn't happen but handle gracefully
      isCase3InProgress = false;
      debug_println("BASE Case 3 completed - unknown type");
      lastLinearCommand = command;
      moveRobot(0, 0);
    }
    return; // Exit early to prevent other command processing
  }

  // Lock system: if case 2 is locked, only allow case 3 to unlock
  if (isCase2Locked && command != 3) {
    debug_println("BASE Case 2 locked - waiting for case 3");
    lastLinearCommand = command;
    moveRobot(0, 0);  // Stop all movement while locked
    return;  // Exit early, ignoring all commands except case 3
  }
  
  // Only move forward when command is 3 (finish cycle command from leg)
  if (command == 3) {
    // Only allow case 3 if we're coming from case 2 (locked state)
    if (!isCase2Locked) {
      debug_println("BASE Case 3 ignored - need case 2 first");
      lastLinearCommand = command;
      moveRobot(0, 0);  // Stop if case 3 is called incorrectly
      return;
    }
    
    // Check what type of case 3 movement based on angular value
    if (angular == 0) {
      // Case 3,0 - Forward movement (existing behavior)
      // Manage transitions for command 3
      if (!wasLinearYActive) {
        debug_println("BASE Starting case 3 forward - unlocking case 2");
        isCase2Locked = false;  // UNLOCK the system
        phaseStartTimeLinear = millis();  // Reset timer for linearY when it becomes active
        wasLinearYActive = true;
        wasAngularActive = false;
        isCase3InProgress = true;  // Mark case 3 as started
        debug_print("BASE Case 3 forward timer started at: ");
        debug_println(phaseStartTimeLinear);
      }
      
      // Calculate elapsed time AFTER timer reset
      unsigned long elapsedTimeLinear = millis() - phaseStartTimeLinear;
      
      // Linear movement profile for command 3 (no delay, immediate forward movement)
      debug_print("BASE Case 3 forward elapsed time: ");
      debug_println(elapsedTimeLinear);
      if (elapsedTimeLinear < 1720) {
        linearY = 16;  // Forward movement for ~1.7 seconds
        debug_println("BASE Case 3 forward movement in progress");
      } else {
        linearY = 0;  // Stop movement after completing the forward cycle
        wasLinearYActive = false;  // Reset the state so it can be triggered again
        isCase3InProgress = false;  // Mark case 3 as completed
        debug_println("BASE Case 3 forward completed");
      }

      lastLinearCommand = command;
      moveRobot(linearY, 0);  // No angular movement for forward case
      
    } else if (angular == 1 || angular == -1) {
      // Case 3,1 or Case 3,-1 - Rotation movement (new behavior)
      // Manage transitions for command 3 rotation
      if (!wasAngularActive) {
        debug_print("BASE Starting case 3 rotation ");
        debug_print(angular > 0 ? "RIGHT" : "LEFT");
        debug_println(" - unlocking case 2");
        isCase2Locked = false;  // UNLOCK the system
        phaseStartTimeAngular = millis();  // Reset timer for angular when it becomes active
        wasAngularActive = true;
        wasLinearYActive = false;
        isCase3InProgress = true;  // Mark case 3 as started
        angularDirectionPositive = angular > 0; // Store direction
        debug_print("BASE Case 3 rotation timer started at: ");
        debug_println(phaseStartTimeAngular);
      }
      
      // Calculate elapsed time AFTER timer reset
      unsigned long elapsedTimeAngular = millis() - phaseStartTimeAngular;
      
      // Rotation movement profile for command 3 (atomic rotation)
      debug_print("BASE Case 3 rotation elapsed time: ");
      debug_println(elapsedTimeAngular);
      if (elapsedTimeAngular < 1720) {
        // Use the same duration as forward movement for consistency
        angular = angularDirectionPositive ? -1.1 : 1.1;  // Full rotation speed
        debug_print("BASE Case 3 rotation ");
        debug_print(angularDirectionPositive ? "RIGHT" : "LEFT");
        debug_println(" movement in progress");
      } else {
        angular = 0;  // Stop rotation after completing the cycle
        wasAngularActive = false;  // Reset the state so it can be triggered again
        isCase3InProgress = false;  // Mark case 3 as completed
        debug_print("BASE Case 3 rotation ");
        debug_print(angularDirectionPositive ? "RIGHT" : "LEFT");
        debug_println(" completed");
      }

      lastLinearCommand = command;
      moveRobot(0, angular);  // No linear movement for rotation case
      
    } else {
      // Invalid angular value for case 3
      debug_print("BASE Case 3 invalid angular value: ");
      debug_println(angular);
      lastLinearCommand = command;
      moveRobot(0, 0);  // Stop for invalid commands
    }
    
  } else if (command == 1 && angular == 0) {
    // Command 1: "Little push" - delay, then brief forward movement, then return to original position
    if (!wasLinearYActive) {
      debug_println("BASE Starting little push");
      phaseStartTimeLinear = millis();  // Reset timer for linearY when it becomes active
      wasLinearYActive = true;
      wasAngularActive = false;
      isCase1InProgress = true;  // Mark case 1 as started
    }
    
    // Calculate elapsed time AFTER timer reset
    unsigned long elapsedTimeLinear = millis() - phaseStartTimeLinear;
    
    // Little push movement profile for command 1
    if (elapsedTimeLinear < 600) {
      linearY = 0;  // First phase: 600ms delay before movement
    } else if (elapsedTimeLinear >= 600 && elapsedTimeLinear < 800) {
      linearY = 12;  // Second phase: 200ms forward movement at moderate speed
    } else if (elapsedTimeLinear >= 800 && elapsedTimeLinear < 1000) {
      linearY = -12;  // Third phase: 200ms backward movement to return to original position
    } else {
      linearY = 0;  // Stop movement after completing the push cycle
      wasLinearYActive = false;  // Reset the state so it can be triggered again
      isCase1InProgress = false;  // Mark case 1 as completed
      debug_println("BASE Little push completed");
    }

    lastLinearCommand = command;
    moveRobot(linearY, angular);
    
  } else if (command == 2) {
    // Command 2: Start timing cycle that blocks all commands (including case 3)
    if (!isCase2InProgress) {
      debug_println("BASE Starting case 2 timing cycle - blocking all commands");
      phaseStartTimeLinear = millis();  // Reset timer for case 2 timing cycle
      wasLinearYActive = true;
      wasAngularActive = false;
      isCase2InProgress = true;  // Mark case 2 timing cycle as started
      isCase2Locked = false;     // Not locked yet, still in progress
      debug_print("BASE Case 2 timer started at: ");
      debug_print(phaseStartTimeLinear);
      debug_print("ms, will complete at: ");
      debug_println(phaseStartTimeLinear + CASE2_CYCLE_TIME);
    }
    
    // Calculate elapsed time AFTER timer reset
    unsigned long elapsedTimeLinear = millis() - phaseStartTimeLinear;
    
    // Case 2: Timing cycle that matches leg timing - no movement, just waiting
    debug_print("BASE Case 2 handler: elapsed=");
    debug_print(elapsedTimeLinear);
    debug_print("ms, target=");
    debug_println(CASE2_CYCLE_TIME);
    
    if (elapsedTimeLinear < CASE2_CYCLE_TIME) {  // Use variable for timing cycle
      linearY = 0;  // No movement during timing cycle
    } else {
      linearY = 0;  // Still no movement
      wasLinearYActive = false;  // Reset the state
      isCase2InProgress = false;  // Mark case 2 timing cycle as completed
      isCase2Locked = true;  // NOW lock the system for case 3
      debug_print("BASE Case 2 handler timing completed after ");
      debug_print(elapsedTimeLinear);
      debug_println("ms - LOCKED for case 3");
    }

    lastLinearCommand = command;
    moveRobot(0, 0);  // No movement during case 2
    
  } else if (command == 0) {
    // Command 0: Idle - Don't interrupt case 2 lock
    if (isCase2Locked) {
      debug_println("BASE Case 2 locked - ignoring idle command");
      lastLinearCommand = command;
      moveRobot(0, 0);
      return;
    }
    
    // Commands 0: Stop the base (no movement)
    wasLinearYActive = false;
    wasAngularActive = false;
    pendingCase3 = false;     // Clear any pending case 3
    pendingAngular = 0.0;     // Clear any pending angular value
    lastLinearCommand = command;
    moveRobot(0, angular);  // Keep angular for turning, but stop linear movement
    
  } else if (angular != 0 && (command == 0 || command == 2 || command == 3)) {
    // Handle angular movement (turning) - works with commands 0, 2, 3 (not 1, as it has its own movement profile)
    if (!wasAngularActive) {
      phaseStartTimeAngular = millis();  // Reset timer for angular when it becomes active
      wasAngularActive = true;
      wasLinearYActive = false;
      angularDirectionPositive = angular > 0; // Store direction of angular movement
    }
    
    // Calculate elapsed time for angular movement
    unsigned long elapsedTimeAngular = millis() - phaseStartTimeAngular;
    
    // Angular movement profile
    if (angularDirectionPositive) {
      if (elapsedTimeAngular < 500) {
        angular = 0.5;  // First phase: 500 ms at 0.5 rad/s
      } else if (elapsedTimeAngular >= 500 && elapsedTimeAngular < 1000) {
        angular = 0;  // Second phase: 500 ms at 0 rad/s
      } else {
        phaseStartTimeAngular = millis();  // Reset cycle after 1 second total
      }
    } else {
      if (elapsedTimeAngular < 500) {
        angular = -0.5;  // First phase: 500 ms at -0.5 rad/s
      } else if (elapsedTimeAngular >= 500 && elapsedTimeAngular < 1000) {
        angular = 0;  // Second phase: 500 ms at 0 rad/s
      } else {
        phaseStartTimeAngular = millis();  // Reset cycle after 1 second total
      }
    }

    moveRobot(0, angular);  // No linear movement, only angular
    
  } else {
    // Default case: stop all movement
    wasLinearYActive = false;
    wasAngularActive = false;
    moveRobot(0, 0);
  }
}

//___________________END_CYTRON_MOTOR________________________//

void reset_all_target_speeds() {
  moveRobot(0, 0);
  last_command_time = millis();
}

void watchdog_tick() {
  current_time = millis();

  if (current_time - last_command_time > MAX_WATCHDOG_ELAPSED_TIME) {
    reset_all_target_speeds();
  }
}

int prevcTime = 0;
bool canWrite = false;
unsigned int lastWriteTime = 0;

// how long we wait for a message to arrive before we write anyway
unsigned int maxWriteElapsed = 5000;
unsigned int now;
unsigned int diff;

unsigned int serial_elapsed;
unsigned int last_serial_time = millis();
#define MIN_SERIAL_ELAPSED 5

bool isMsg = false;

// Add this to send periodic status updates
void send_status_update() {
  unsigned long currentTime = millis();
  
  // Check if it's time to send a status update
  if (currentTime - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = currentTime;
    
    // Send basic status information
    sprintf(feedback_msg, "BASE_ALIVE: UPTIME:%lu L_ENC:%ld R_ENC:%ld SPD_L:%d SPD_R:%d", 
        millis()/1000, rightEncoder.read(), leftEncoder.read(), PWM_val[0], PWM_val[1]);
    debug_println(feedback_msg);
  }
}

// Process special commands like PING
void process_special_command(char* buffer) {
  if (strstr(buffer, "PING:1") != NULL) {
    // Respond to ping with a status message
    sprintf(feedback_msg, "PONG:1 UPTIME:%lu", millis()/1000);
    Serial.println(feedback_msg);
    
    // Also send additional status information
    sprintf(feedback_msg, "STATUS: L:%ld R:%ld SPD_L:%.2f SPD_R:%.2f", 
        leftEncoder.read(), rightEncoder.read(), speed_act[0], speed_act[1]);
    Serial.println(feedback_msg);
  } else if (strstr(buffer, "HB:1") != NULL) {
    // Respond to heartbeat with more info
    sprintf(feedback_msg, "HB:ACK MOTORS:%.2f,%.2f PWM:%d,%d", 
        lastSpeedCommand[0], lastSpeedCommand[1], PWM_val[0], PWM_val[1]);
    Serial.println(feedback_msg);
  } else if (strstr(buffer, "STATUS") != NULL) {
    // Respond to status request with detailed information
    sprintf(feedback_msg, "BASE_STATUS: L_POS:%ld R_POS:%ld L_SPD:%.2f R_SPD:%.2f", 
        leftEncoder.read(), rightEncoder.read(), speed_act[0], speed_act[1]);
    Serial.println(feedback_msg);
    
    // Second line with PWM and target speeds
    sprintf(feedback_msg, "TARGET: L_REQ:%.2f R_REQ:%.2f L_PWM:%d R_PWM:%d", 
        speed_req[0], speed_req[1], PWM_val[0], PWM_val[1]);
    Serial.println(feedback_msg);
  }
}

//Set values to motors
void serial_loop() {
  static unsigned long lastErrorTime = 0;
  static int errorCount = 0;
  
  // a serial loop can only be performed once every MIN_SERIAL_ELAPSED time
  now = millis();

  serial_elapsed = now - last_serial_time;
  if (serial_elapsed < MIN_SERIAL_ELAPSED)
    return;

  // Add safety check - if we're getting too many errors, throttle processing
  if (errorCount > 10 && (millis() - lastErrorTime < 1000)) {
    delay(100); // Add delay to slow down
    return;
  }

  // read everything it can from serial
  while (read_key_value_serial() && !isMsg) {
    isMsg = true;
  }

  // if at least a msg was received, use the updated values
  if (isMsg) {
    // Print received command values for debugging
    // debug_print("BASE DEBUG - Received: BF:");
    // debug_print(lastSpeedCommand[0]);
    // debug_print(" BB:");
    // debug_println(lastSpeedCommand[1]);
    
    // Validate speed values before using them - add bounds checking
    if (isnan(lastSpeedCommand[0]) || isnan(lastSpeedCommand[1]) ||
        isinf(lastSpeedCommand[0]) || isinf(lastSpeedCommand[1])) {
      debug_println("ERROR: Invalid speed values detected!");
      lastSpeedCommand[0] = 0;
      lastSpeedCommand[1] = 0;
      errorCount++;
      lastErrorTime = millis();
    } else {
      // // Print which command path we're taking
      // if (lastSpeedCommand[0] != 0 || lastSpeedCommand[1] != 0) {
      //   debug_println("Executing: Movement Command");
      //   // Also send info about current encoder positions for debugging
      //   sprintf(feedback_msg, "POS: L:%ld R:%ld SPD: L:%.2f R:%.2f", 
      //     leftEncoder.read(), rightEncoder.read(), speed_act[0], speed_act[1]);
      //   debug_println(feedback_msg);
      // }
      updateBaseMovementByTime(lastSpeedCommand[0], lastSpeedCommand[1]);
    }
    
    canWrite = true;
    last_serial_time = millis();
    isMsg = false;
  }
  
  // Run PID loop on every iteration
  pidLoop();
}

void setup() {
  // Disable watchdog first thing to prevent reset loops
  wdt_disable();
  
  // Start communication with a brief delay to stabilize
  Serial.begin(115200);
  delay(500); // Longer delay for stability
  Serial.println("=====SYSTEM STARTING=====");
  
  // Report reset cause
  uint8_t resetFlags = MCUSR;
  MCUSR = 0; // Clear reset flags
  
  if (resetFlags & (1<<WDRF)) {
    Serial.println("WARNING: Reset caused by Watchdog Timer");
  }
  if (resetFlags & (1<<BORF)) {
    Serial.println("WARNING: Reset caused by Brown-out");
  }
  if (resetFlags & (1<<EXTRF)) {
    Serial.println("INFO: Reset caused by external reset button");
  }
  if (resetFlags & (1<<PORF)) {
    Serial.println("INFO: Power-on reset");
  }
  
  canWrite = false;
  lastWriteTime = millis();

  Serial.setTimeout(5000);
  prevcTime = millis();

  // Initialize encoder positions
  leftEncoder.write(0);
  rightEncoder.write(0);
  
  // Initialize encoder tracking variables
  prevLeftPosition = 0;
  prevRightPosition = 0;
  
  // Initialize PID variables
  for (int i = 0; i < 2; i++) {
    speed_req[i] = 0;
    speed_act[i] = 0;
    PWM_val[i] = 0;
    last_error[i] = 0;
    Iterm[i] = 0;
    lastSP[i] = 0;
    lastMillis[i] = millis();
  }
  lastMilliLoop = millis();
  
  // Initialize lastStatusUpdate time
  lastStatusUpdate = millis();
  
  // Better startup messages similar to the head Arduino
  debug_println("Arduino BASE setup started");
  debug_println("Setup BASE Complete");
}

void loop() {
  watchdog_tick();
  serial_loop();
  send_status_update();
    
  // Periodically report memory usage
  static unsigned long lastMemReport = 0;
  if (millis() - lastMemReport > 10000) { // Every 10 seconds
    lastMemReport = millis();
      
    // Calculate available memory
    extern int __heap_start, *__brkval;
    int freeMemory;
      
    if((int)__brkval == 0)
      freeMemory = ((int)&freeMemory) - ((int)&__heap_start);
    else
      freeMemory = ((int)&freeMemory) - ((int)__brkval);
      
    debug_print("Available memory: ");
    debug_println(freeMemory);
  }
}
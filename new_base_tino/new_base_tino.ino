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
  unsigned long elapsedTimeLinear = currentTime - phaseStartTimeLinear;
  unsigned long elapsedTimeAngular = currentTime - phaseStartTimeAngular;

  // Manage transitions
  if (linearY != 0 && !wasLinearYActive && angular == 0) {
    phaseStartTimeLinear = millis();  // Reset timer for linearY when it becomes active
    wasLinearYActive = true;
    wasAngularActive = false;
  } else if (angular != 0 && !wasAngularActive && linearY == 0) {
    phaseStartTimeAngular = millis();  // Reset timer for angular when it becomes active
    wasAngularActive = true;
    wasLinearYActive = false;
    angularDirectionPositive = angular > 0; // Store direction of angular movement
  } else if (linearY == 0 && angular == 0) {
    wasLinearYActive = false;
    wasAngularActive = false;
  }

  if (linearY != 0 && angular == 0) {
    // Linear movement profile
    if (elapsedTimeLinear < 1300) {
      linearY = 0;  // First phase: 1.3 seconds pause
    } else if (elapsedTimeLinear >= 1300 && elapsedTimeLinear < 3020) {
      linearY = 16;  // Second phase: ~1.7 seconds forward movement
    } else {
      phaseStartTimeLinear = millis();  // Reset cycle after 3.02 seconds total
    }

    moveRobot(linearY, angular);

  } else if (angular != 0 && linearY == 0) {
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

    moveRobot(linearY, angular);

  } else if (linearY != 0 && angular != 0) {
    // Handle intermediate situation: when both are non-zero from the start
    if (!wasLinearYActive && !wasAngularActive) {
      linearY = 0;
      angular = 0;
    } else if (wasLinearYActive && !wasAngularActive) {
      angular = 0;  // If linearY was active and angular becomes non-zero, continue with linearY
    } else if (wasAngularActive && !wasLinearYActive) {
      linearY = 0;  // If angular was active and linearY becomes non-zero, continue with angular
    }
    moveRobot(linearY, angular);
  } else {
    moveRobot(linearY, angular);
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
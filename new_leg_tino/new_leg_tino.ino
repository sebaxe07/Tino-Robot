#include <ezButton.h>
#include <math.h>
#include "RaspberryCommunication.h"
#include <CytronMotorDriver.h>

#define INIT_KP 0.8
#define INIT_KI 0.04
#define INIT_KD 0.03

#define TICKS_PER_ROTATION 2000.0

// #define _2A 6   // Pin di controllo del motore //2B //arancione
// #define _2B 5   // Pin di controllo del motore //2A //blu

#define _2A 10  // Pin di controllo del motore //2B prima 6 //arancione
#define _2B 9   // Pin di controllo del motore //2A prima 5 //blu

#define ENCA 3
#define ENCB 2

ezButton button(13);
// Configurazione del driver motore - PWM_PWM mode for proper negative PWM handling
CytronMD motor(PWM_PWM, _2A, _2B);  // Motore: _2A = backward PWM, _2B = forward PWM

int state = 0;

//______________PID
float vt = 0;
float SampleTime = 20.0;
int outMax = 200;
int outMin = -200;
volatile int ticks = 0;
volatile int absolutePosition = 0; // Track absolute encoder position
float prevTime = 0;
float ticksPerMillisec = 0;
float rpm = 0;
float rpmFilt = 0;
float rpmPrev = 0;

float ITerm = 0;
float kp, ki, kd, lastV;

//______________setup
bool canWrite = false;
unsigned int lastWriteTime = 0;
unsigned int now;
unsigned int serial_elapsed;
unsigned int min_serial_elapsed = 5;
unsigned int last_serial_time = millis();
bool isMsg = false;

// Debug control - Change this to true to enable debug output
#define DEBUG_ENABLED false  // Set to true to enable debug output
#define DEBUG_PRINT(x) if(DEBUG_ENABLED) Serial.print(x)
#define DEBUG_PRINTLN(x) if(DEBUG_ENABLED) Serial.println(x)

// Uncomment the line below to enable debug mode
// #define DEBUG_ENABLED true
//_______________
bool isSinusoidalActive = false;
unsigned long startTime = millis();  // Momento iniziale del ciclo sinusoidale
static float cycleTime = 3.1;        // Tempo totale del ciclo in secondi, valore iniziale di default
//_______________
// New behavior states
bool isLittlePushActive = false;
bool isForwardOnlyActive = false;
bool isFinishingCycle = false;
bool littlePushCompleted = false;  // Track if little push just completed
unsigned long littlePushStartTime = 0;
float littlePushDuration = 0.8;  // Reduced duration from 1.0 to 0.8 seconds
bool isCase2Locked = false;  // Lock system for case 2
bool isReturningFromCase2 = false;  // Flag to track return movement from case 2
bool pendingCase3 = false;         // Flag to track if case 3 is waiting for case 2 to complete

// State tracking for atomic movements that must complete
bool isCase1InProgress = false;    // Tracks if case 1 "little push" is in progress
bool isCase2InProgress = false;    // Tracks if case 2 movement sequence is in progress
bool isCase3InProgress = false;    // Tracks if case 3 movement sequence is in progress
unsigned long case3ReturnStartTime = 0;  // Track when case 3 return movement started
float case3ReturnDuration = 2.0;  // Maximum time for return movement (2 seconds)
//_______________
bool previousSteadyState = 0;
bool unpressedState = 1;  // HIGH
bool pressedState = 0;    // LOW
bool isPressed = false;

//_______________

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    ticks++;
    absolutePosition++;
  } else {
    ticks--;
    absolutePosition--;
  }
}

float pid(float vt, float v) {
  // Zona morta: se vt è vicino a zero, imposta l'uscita del PID a zero
  if (abs(vt) < 5 && abs(v) < 5) {
    ITerm = 0;
    lastV = v;
    return 0;  // Imposta l'uscita a zero per evitare oscillazioni
  }
  float error = vt - v;
  ITerm += (ki * error);
  if (ITerm > outMax) {
    ITerm = outMax;
  } else if (ITerm < outMin) {
    ITerm = outMin;
  }
  double dInput = (v - lastV);
  float u = kp * error + ITerm - kd * dInput;
  if (u > outMax) {
    u = outMax;
  } else if (u < outMin) {
    u = outMin;
  }
  lastV = v;

  if (abs(vt) < 5) {
    ITerm = 0;  // Resetta l'integrale per evitare windup
    u = 0;      // Forza l'uscita a zero per fermare il motore
  }
  return u;
}

// state == HIGH --> UNTOUCHED == 1 || state == LOW --> TOUCHED == 0
bool ButtonisPressed() {
  if (previousSteadyState == unpressedState && state == pressedState) {
    previousSteadyState = state;
    return true;
  } else {
    previousSteadyState = state;
    return false;
  }
}

// Function to calculate the little push profile
float vtLittlePush() {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - littlePushStartTime) / 1000.0;  // Tempo trascorso in secondi
  
  // Only print debug info occasionally during execution
  static unsigned long lastLittlePushPrint = 0;
  bool shouldPrint = DEBUG_ENABLED && (currentTime - lastLittlePushPrint > 500);  // Print every 500ms max
  
  if (elapsedTime < littlePushDuration * 0.5) {
    // First 50%: move forward 
    float progress = elapsedTime / (littlePushDuration * 0.5);
    float targetRPM = 90.0 + 80.0 * progress;
    if (shouldPrint) {
      DEBUG_PRINT("Little Push FORWARD - Elapsed: "); DEBUG_PRINT(elapsedTime); 
      DEBUG_PRINT("s, Target: "); DEBUG_PRINTLN(targetRPM);
      lastLittlePushPrint = currentTime;
    }
    return targetRPM;
  } else if (elapsedTime < littlePushDuration * 0.55) {
    // Brief 5% stop phase - minimal braking time
    if (shouldPrint) {
      DEBUG_PRINTLN("Little Push STOPPING");
      lastLittlePushPrint = currentTime;
    }
    return 0.0;
  } else if (elapsedTime < littlePushDuration) {
    // Last 45%: move backward effectively
    float returnPhase = (elapsedTime - (littlePushDuration * 0.55)) / (littlePushDuration * 0.45);
    float targetRPM = 0 - 255.0 * returnPhase;  // Go down to -255 RPM (more effective backward movement)
    if (shouldPrint) {
      DEBUG_PRINT("Little Push RETURN - Target: "); DEBUG_PRINTLN(targetRPM);
      lastLittlePushPrint = currentTime;
    }
    return targetRPM;
  } else {
    // Movement complete - force stop
    DEBUG_PRINTLN("Little push COMPLETE");
    isLittlePushActive = false;
    return 90.0;
  }
}

// Function to calculate the forward-only sinusoidal profile (goes to full 160)
float vtForwardOnly() {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;  // Tempo trascorso in secondi

  // Calcolo della fase del ciclo (tra 0 e valore corrente di cycleTime)
  float phase = fmod(elapsedTime, cycleTime);

  // Funzione sinusoidale modificata per raggiungere il picco completo
  if (phase < (1.0 / cycleTime) * cycleTime) {
    // Salita moderata da 90 a 100
    return 90.0 + 10.0 * (1 - exp(-3 * phase));
  } else if (phase < (1.5 / cycleTime) * cycleTime) {
    // Salita rapida da 100 a 160 (picco completo)
    return 100.0 + 60.0 * (1 - exp(-10 * (phase - (1.0 / cycleTime) * cycleTime)));
  } else {
    // Reached maximum position - activate lock
    isCase2Locked = true;
    isForwardOnlyActive = false;
    DEBUG_PRINTLN("Forward movement completed - LOCKED");
    return 0;  // Stop and lock
  }
}

// Function to calculate the sinusoidal profile
float vtSinusoidal() {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0;  // Tempo trascorso in secondi

  // Tempo totale del ciclo in secondi
  float amplitude = 35.0;  // Ampiezza della variazione da 90
  float offset = 125.0;    // Valore medio della funzione (offset)

  // Calcolo della fase del ciclo (tra 0 e valore corrente di cycleTime)
  float phase = fmod(elapsedTime, cycleTime);

  // Funzione sinusoidale modificata per seguire il profilo desiderato
  if (phase < (1.0 / cycleTime) * cycleTime) {
    // Salita moderata da 90 a 100
    return 90.0 + 10.0 * (1 - exp(-3 * phase));
  } else if (phase < (1.5 / cycleTime) * cycleTime) {
    // Picco rapido da 100 a 160
    return 100.0 + 80.0 * (1 - exp(-10 * (phase - (1.0 / cycleTime) * cycleTime)));
  } else if (phase < cycleTime) {
    // Discesa lenta da 160 a 90
    return 160.0 - 70.0 * ((phase - (1.5 / cycleTime) * cycleTime) / ((2.5 / cycleTime) * cycleTime));
  }

  return offset;  // Valore di default in caso di condizioni non soddisfatte
}

void moveLeg(float linearX, float angular) {
  button.loop();
  state = button.getState();
  static float lastVt = 0;  // Store the last value of vt
  static int lastLinearX = 0;  // Store the last linearX command

  // Convert linearX to integer for switch statement
  int command = (int)linearX;
  
  // Debug: Print command received (only when it changes)
  if (DEBUG_ENABLED && command != lastLinearX) {
    DEBUG_PRINT("Command: "); DEBUG_PRINT(lastLinearX); 
    DEBUG_PRINT(" -> "); DEBUG_PRINTLN(command);
  }

  // Check if case 1 is in progress and must complete
  if (isCase1InProgress) {
    if (isLittlePushActive) {
      vt = vtLittlePush();
    } else {
      // Case 1 completed
      isCase1InProgress = false;
      if (!littlePushCompleted) {
        DEBUG_PRINTLN("Case 1 completed");
        littlePushCompleted = true;
      }
      vt = 0;  // Stop after completion
    }
    lastLinearX = command;
    return;  // Exit early to prevent other command processing
  }
  
  // Check if case 2 is in progress and must complete
  if (isCase2InProgress) {
    // Check if case 3 command is received during case 2 - save it for later
    if (command == 3 && !pendingCase3) {
      pendingCase3 = true;
      DEBUG_PRINTLN("LEG Case 3 received during case 2 - saving for later execution");
    }
    
    if (isForwardOnlyActive) {
      vt = constrain(vtForwardOnly(), 0, 180);
    } else {
      // Case 2 completed - it should be locked now
      isCase2InProgress = false;
      DEBUG_PRINTLN("LEG Case 2 completed - LOCKED");
      
      // If case 3 was pending, automatically execute it now
      if (pendingCase3) {
        DEBUG_PRINTLN("LEG Executing pending case 3 after case 2 completion");
        pendingCase3 = false;  // Clear the pending flag
        isCase2Locked = false;  // UNLOCK the system
        isCase3InProgress = true;  // Mark case 3 as started
        isReturningFromCase2 = true;  // Start return movement
        case3ReturnStartTime = millis();  // Initialize return timer
        vt = 90;  // Start returning to neutral position
        DEBUG_PRINTLN("LEG Auto case 3 return movement started");
      } else {
        vt = 0;  // Stop and lock if no pending case 3
      }
    }
    lastLinearX = command;
    return;  // Exit early to prevent other command processing
  }
  
  // Check if case 3 is in progress and must complete  
  if (isCase3InProgress) {
    // Handle case 3 atomic completion
    if (isReturningFromCase2) {
      unsigned long currentTime = millis();
      float elapsedReturnTime = (currentTime - case3ReturnStartTime) / 1000.0;
      
      // Only print occasionally during return movement
      static unsigned long lastCase3Print = 0;
      if (DEBUG_ENABLED && currentTime - lastCase3Print > 1000) {  // Print every 1000ms max
        DEBUG_PRINT("Case 3 return - elapsed: "); DEBUG_PRINT(elapsedReturnTime); 
        DEBUG_PRINT("s, RPM: "); DEBUG_PRINT(rpmFilt);
        DEBUG_PRINT(", Button: "); DEBUG_PRINT(state == LOW ? "PRESSED" : "NOT_PRESSED");
        DEBUG_PRINTLN("");
        lastCase3Print = currentTime;
      }
      
      vt = 90;  // Continue returning to neutral
      
      // Check completion criteria: Only button press (most reliable indicator)
      bool buttonPressed = (state == LOW);  // Button pressed means home position reached
      
      if (buttonPressed) {
        DEBUG_PRINT("Case 3 return complete - Button pressed, time: "); 
        DEBUG_PRINT(elapsedReturnTime); DEBUG_PRINTLN("s");
        isReturningFromCase2 = false;
        isCase3InProgress = false;
        vt = 0;  // Stop at home position
      }
    } else if (isSinusoidalActive || isForwardOnlyActive || isLittlePushActive) {
      // Continue any active movement until completion
      if (isSinusoidalActive) {
        vt = constrain(vtSinusoidal(), 0, 180);
        unsigned long currentTime = millis();
        float elapsedTime = (currentTime - startTime) / 1000.0;
        if (elapsedTime >= cycleTime) { 
          isSinusoidalActive = false;
          isCase3InProgress = false;
          DEBUG_PRINTLN("Case 3 sinusoidal complete");
        }
      } else if (isForwardOnlyActive) {
        vt = constrain(vtForwardOnly(), 0, 180);
        if (!isForwardOnlyActive) {
          isCase3InProgress = false;
          DEBUG_PRINTLN("Case 3 forward complete");
        }
      } else if (isLittlePushActive) {
        vt = vtLittlePush();  // Allow negative values for backward movement
        if (!isLittlePushActive) {
          isCase3InProgress = false;
          DEBUG_PRINTLN("Case 3 little push complete");
        }
      }
    } else {
      // No active movement, case 3 completed
      isCase3InProgress = false;
      DEBUG_PRINTLN("Case 3 complete");
      vt = 0;
    }
    lastLinearX = command;
    return;  // Exit early to prevent other command processing
  }

  // Lock system: if case 2 is locked, only allow case 3 to unlock
  if (isCase2Locked && command != 3) {
    DEBUG_PRINTLN("LEG Case 2 locked - waiting for case 3");
    vt = 0;  // Stop at locked position
    lastLinearX = command;
    return;  // Exit early, ignoring all commands except case 3
  }

  switch (command) {
    case 0:  // Idle
      // Don't interrupt case 3 if it's in progress - let it complete atomically
      if (isCase3InProgress) {
        DEBUG_PRINTLN("Case 3 in progress - ignoring idle command");
        lastLinearX = command;
        return;
      }
      
      // Reset case 2 return flag when going to idle
      if (isReturningFromCase2) {
        DEBUG_PRINTLN("LEG Idle - stopping return");
        isReturningFromCase2 = false;
        isCase3InProgress = false;  // Complete case 3 when reaching idle
      }
      
      // Reset all atomic movement flags when going to idle
      isCase1InProgress = false;
      isCase2InProgress = false;
      isCase3InProgress = false;
      pendingCase3 = false;  // Clear any pending case 3
      
      // Deactivate all movement patterns
      if (isSinusoidalActive) {
        isSinusoidalActive = false;
      }
      if (isLittlePushActive) {
        isLittlePushActive = false;
      }
      if (isForwardOnlyActive) {
        isForwardOnlyActive = false;
      }
      isFinishingCycle = false;
      littlePushCompleted = false;  // Reset completion flag for future pushes

      // AUTO BUTTON SYSTEM - ONLY ACTIVE IN IDLE MODE
      if (state == HIGH) {
        // Button not pressed - move towards start position intelligently
        // Use absolute encoder position to determine which direction to move
        // Assuming neutral/start position is 0 ticks (where button is pressed)
        int targetPosition = 0;  // Start position in ticks
        int currentPosition = absolutePosition;
        int positionDifference = targetPosition - currentPosition;
        
        // If we're close to target (within 50 ticks), stay at neutral RPM
        if (abs(positionDifference) < 50) {
          vt = 90.0;
          // Only print occasionally when in stable position
          static unsigned long lastIdlePrint = 0;
          if (DEBUG_ENABLED && millis() - lastIdlePrint > 5000) {  // Print every 5 seconds when stable
            DEBUG_PRINT("Near start pos - diff: "); DEBUG_PRINT(positionDifference); 
            DEBUG_PRINT(", abs: "); DEBUG_PRINTLN(absolutePosition);
            lastIdlePrint = millis();
          }
        } else {
          // Move towards target based on position difference
          // If current position is positive (moved forward from start), need to move backward
          // If current position is negative (moved backward from start), need to move forward
          static int lastPrintedDiff = -9999;
          bool shouldPrintDirection = DEBUG_ENABLED && abs(positionDifference - lastPrintedDiff) > 50;
          
          if (currentPosition > 0) {
            // Current position is positive (forward from start), need to move backward (negative RPM)
            vt = -20;  // Move backward with negative RPM
            if (shouldPrintDirection) {
              DEBUG_PRINT("Moving backward - pos: "); DEBUG_PRINT(currentPosition); DEBUG_PRINT(", diff: "); DEBUG_PRINTLN(positionDifference);
              lastPrintedDiff = positionDifference;
            }
          } else {
            // Current position is negative (backward from start), need to move forward (higher RPM)
            vt = 110.0;  // Move forward with positive RPM
            if (shouldPrintDirection) {
              DEBUG_PRINT("Moving forward - pos: "); DEBUG_PRINT(currentPosition); DEBUG_PRINT(", diff: "); DEBUG_PRINTLN(positionDifference);
              lastPrintedDiff = positionDifference;
            }
          }
          
          // No need to constrain since we're using specific values
          // vt = constrain(vt, 60, 120);  // Removed - let negative values through
        }
      } else if (state == LOW) {
        vt = 0;   // Stop at limit switch
        // Reset absolute position when button is pressed AND leg is actually stopped
        // Only reset if we're not currently in the middle of a movement
        static unsigned long lastResetTime = 0;
        unsigned long currentTime = millis();
        
        if (absolutePosition != 0 && 
            abs(rpmFilt) < 10.0 &&  // Only reset when RPM is low (nearly stopped)
            (currentTime - lastResetTime > 1000)) {  // Prevent rapid resets (min 1 second between resets)
          absolutePosition = 0;
          lastResetTime = currentTime;
          DEBUG_PRINT("Reset abs pos (RPM: "); DEBUG_PRINT(rpmFilt); DEBUG_PRINTLN(")");
        }
      }
      break;

    case 1:  // Little push
      if (!isLittlePushActive && lastLinearX != 1) {
        // Start little push movement
        DEBUG_PRINTLN("Starting little push");
        isLittlePushActive = true;
        isCase1InProgress = true;  // Mark case 1 as started
        littlePushCompleted = false;  // Reset completion flag
        littlePushStartTime = millis();
        // Deactivate other movements
        isSinusoidalActive = false;
        isForwardOnlyActive = false;
        isFinishingCycle = false;
      }
      
      if (isLittlePushActive) {
        vt = vtLittlePush();  // Remove constrain to allow negative values
      } else {
        // Little push completed, stay at idle
        if (!littlePushCompleted) {
          DEBUG_PRINTLN("Little push complete");
          littlePushCompleted = true;
        }
        // Set leg to stop after little push completion
        vt = 0;  // Force stop when little push is done
      }
      break;

    case 2:  // Forward until maximum reach (goes to full 160)
      if (!isForwardOnlyActive && lastLinearX != 2) {
        // Start forward-only movement
        DEBUG_PRINTLN("LEG Starting forward movement");
        isForwardOnlyActive = true;
        isCase2InProgress = true;  // Mark case 2 as started
        startTime = millis();
        // Deactivate other movements
        isSinusoidalActive = false;
        isLittlePushActive = false;
        isFinishingCycle = false;
      }
      
      if (isForwardOnlyActive) {
        vt = constrain(vtForwardOnly(), 0, 180);
      } else {
        // Forward movement completed, stay at idle
        vt = 0;  // Force stop when forward movement is done
      }
      break;

    case 3:  // Finish sinusoidal movement and stop at idle - also unlocks case 2
      // Only allow case 3 if we're coming from case 2 (locked or returning)
      if (!isCase2Locked && !isReturningFromCase2) {
        DEBUG_PRINTLN("Case 3 ignored - need case 2 first");
        vt = 0;  // Stop if case 3 is called incorrectly
        break;  // Ignore case 3 if not coming from case 2
      }
      
      // Mark case 3 as started for atomic execution
      if (!isCase3InProgress) {
        DEBUG_PRINTLN("LEG Starting case 3");
        isCase3InProgress = true;
      }
      
      // Check if we need to unlock case 2
      if (isCase2Locked) {
        DEBUG_PRINT("LEG Unlocking case 2 - current RPM: "); DEBUG_PRINTLN(rpmFilt);
        isCase2Locked = false;
        isForwardOnlyActive = false;
        isReturningFromCase2 = true;  // Start return movement
        case3ReturnStartTime = millis();  // Initialize return timer
        vt = 90;  // Go back to neutral position after unlock
        break;
      }
      
      // Check if we're returning from case 2
      if (isReturningFromCase2) {
        DEBUG_PRINTLN("LEG Continuing return to neutral");
        vt = 90;  // Continue moving back to neutral
        // Keep returning until completion (handled in case 3 progress section above)
        break;
      }
      
      if (isSinusoidalActive || isForwardOnlyActive || isLittlePushActive) {
        // If any movement is active, let it finish naturally
        isFinishingCycle = true;
        
        if (isSinusoidalActive) {
          vt = constrain(vtSinusoidal(), 0, 180);
          // Check if we completed the cycle
          unsigned long currentTime = millis();
          float elapsedTime = (currentTime - startTime) / 1000.0;
          if (elapsedTime >= cycleTime) {
            isSinusoidalActive = false;
            isFinishingCycle = false;
          }
        } else if (isForwardOnlyActive) {
          vt = constrain(vtForwardOnly(), 0, 180);
          if (!isForwardOnlyActive) {  // vtForwardOnly() sets this to false when done
            isFinishingCycle = false;
          }
        } else if (isLittlePushActive) {
          vt = constrain(vtLittlePush(), 0, 180);
          if (!isLittlePushActive) {  // vtLittlePush() sets this to false when done
            isFinishingCycle = false;
          }
        }
      } else {
        // No active movement, stay at idle
        vt = 0;  // Force stop when no movement is active
      }
      break;

    default:  // Any other value - treat as old behavior for compatibility
      if (linearX != 0 && angular == 0) {
        if (!isSinusoidalActive) {
          // Attiva il profilo sinusoidale e resetta startTime
          isSinusoidalActive = true;
          startTime = millis();  // Resetta il tempo di inizio del ciclo
          // Deactivate other movements
          isLittlePushActive = false;
          isForwardOnlyActive = false;
          isFinishingCycle = false;
        }
        vt = constrain(vtSinusoidal(), 0, 180);
        lastVt = vt;

        // Resetta il ciclo se il pulsante è premuto
        if (isPressed) {
          float elapsedTime = (millis() - startTime) / 1000.0;
          DEBUG_PRINT("Button pressed - elapsed: "); DEBUG_PRINTLN(elapsedTime);
          cycleTime = constrain(elapsedTime, 3.0, 4.0);
          startTime = millis();
        }
      } else if (linearX == 0 && angular == 0) {
        // Stop behavior
        if (isSinusoidalActive) {
          isSinusoidalActive = false;
        }
        if (isLittlePushActive) {
          isLittlePushActive = false;
        }
        if (isForwardOnlyActive) {
          isForwardOnlyActive = false;
        }
        isFinishingCycle = false;

        vt = 0;  // Force stop in default stop behavior
      } else if (linearX != 0 && angular != 0) {
        // Turn behavior - simplified without auto button system
        vt = 120;  // Standard turn speed
      }
      break;
  }

  lastLinearX = command;  // Store current command for next iteration
}

void computeOutput() {
  float currTime = millis();
  if (currTime - prevTime < SampleTime) {
    return;
  }
  float deltaT = currTime - prevTime;

  ticksPerMillisec = (float)ticks / deltaT;
  prevTime = currTime;
  ticks = 0;

  rpm = (60000.0 * ticksPerMillisec) / TICKS_PER_ROTATION;
  rpmFilt = 0.95 * rpmFilt + 0.025 * rpm + 0.025 * rpmPrev;  // 8hz
  rpmPrev = rpm;

  float u = pid(vt, rpmFilt);

  // Selective debug output - only print when there are significant changes
  static float lastPrintedVt = -999;
  static float lastPrintedRpm = -999;
  static int lastPrintedAbsPos = -9999;
  static bool lastLittleState = false;
  static bool lastForwardState = false;
  static bool lastSinusoidalState = false;
  static unsigned long lastDebugPrint = 0;
  
  bool shouldPrint = false;
  
  // Print if any significant change occurred
  if (abs(vt - lastPrintedVt) > 2.0 ||
      abs(rpmFilt - lastPrintedRpm) > 5.0 ||
      abs(absolutePosition - lastPrintedAbsPos) > 20 ||
      isLittlePushActive != lastLittleState ||
      isForwardOnlyActive != lastForwardState ||
      isSinusoidalActive != lastSinusoidalState ||
      (currTime - lastDebugPrint > 2000)) {  // Force print every 2 seconds
    shouldPrint = true;
  }
  
  if (shouldPrint) {
    // Serial.print("Abs Pos: ");
    // Serial.print(absolutePosition);
    // Serial.print(", Target RPM: ");
    // Serial.print(vt);
    // Serial.print(", Actual RPM: ");
    // Serial.print(rpmFilt);
    // Serial.print(", PWM: ");
    // Serial.print((int)u);
    // Serial.print(", States - Little: ");
    // Serial.print(isLittlePushActive ? "ON" : "OFF");
    // Serial.print(", Forward: ");
    // Serial.print(isForwardOnlyActive ? "ON" : "OFF");
    // Serial.print(", Sinusoidal: ");
    // Serial.print(isSinusoidalActive ? "ON" : "OFF");
    // Serial.print(", Button: ");
    // Serial.println(state == HIGH ? "HIGH" : "LOW");
    
    // Update last printed values
    lastPrintedVt = vt;
    lastPrintedRpm = rpmFilt;
    lastPrintedAbsPos = absolutePosition;
    lastLittleState = isLittlePushActive;
    lastForwardState = isForwardOnlyActive;
    lastSinusoidalState = isSinusoidalActive;
    lastDebugPrint = currTime;
  }

  int pwm = (int)(u);
  // PWM_PWM mode handles negative values properly, no need to constrain to positive range
  pwm = constrain(pwm, -255, 255);
  motor.setSpeed(pwm);
}

void serial_loop() {
  now = millis();
  serial_elapsed = now - last_serial_time;
  // if (serial_elapsed < min_serial_elapsed)
  //   return;
  while (read_key_value_serial() && !isMsg) {
    isMsg = true;
  }
  if (isMsg) {
    // moveLeg(lastSpeedCommand[0]);
    // DEBUG_PRINT(lastSpeedCommand[0]);
    canWrite = true;
    last_serial_time = millis();
    isMsg = false;
  }
  moveLeg(lastSpeedCommand[0], lastSpeedCommand[1]);
  // DEBUG_PRINTLN(lastSpeedCommand[0]);
}

void setup() {
  Serial.begin(115200);
  Serial.flush();
  delay(1000);

  button.setDebounceTime(200);

  // Imposta il prescaler di Timer 1 a 1 per ottenere ~31.25 kHz sui pin 9 e 10
  TCCR1B = TCCR1B & 0b11111000 | 0x01;  // Prescaler = 1

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  kp = INIT_KP;
  ki = INIT_KI;
  kd = INIT_KD;

  // Initial position check
  button.loop();
  state = button.getState();
  if (state == HIGH) {
    vt = 120;  // Move towards initial position
    while (state == HIGH) {
      computeOutput();
      button.loop();
      state = button.getState();
    }
    vt = 0;  // Stop when the button is pressed (initial position reached)
    absolutePosition = 0;  // Reset absolute position at start position
    DEBUG_PRINTLN("Reset abs pos at start");
  }

  canWrite = false;
  lastWriteTime = millis();
  write_serial(LEG_OK_FLAG);
  Serial.setTimeout(5000);
  prevTime = millis();

  moveLeg(0, 0);

  Serial.println("Arduino LEG setup complete");
}

void loop() {
  button.loop();
  state = button.getState();  // state == HIGH --> UNTOUCHED == 1 || state == LOW --> TOUCHED == 0
                              //codice giusto

  isPressed = ButtonisPressed();
  serial_loop();
  computeOutput();
}
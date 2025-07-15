#include <Servo.h>
#include "RaspberryCommunication.h"

// Debug flag - set to true to enable debug prints, false to disable
#define DEBUG_MODE false

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

#define PIN_SERVO_A 7
#define PIN_SERVO_B 8
#define PIN_SERVO_C 9

#define PIN_POT_A A0
#define PIN_POT_B A1
#define PIN_POT_C A2

int a_min = 1220;  //90°
int b_min = 1220;
int c_min = 1220;

int theta45 = 335;  //45°

int a_max = a_min + theta45 * 2;  //0°
int b_max = b_min + theta45 * 2;
int c_max = c_min + theta45 * 2;


Servo servoA;
Servo servoB;
Servo servoC;

float k;

// int range;
int forward_angle;

int a, b, c;
int a_0, b_0, c_0;
int b_bl, c_br;
int a_fb, b_fb, c_fb;

int thetaA;
int thetaB;
int thetaC;

int a_deg;
int b_deg;
int c_deg;


//________RASP_COMM________//

#define MAX_WATCHDOG_ELAPSED_TIME 100000

void reset_all_target_speeds() {
  servoA.writeMicroseconds(a_min + theta45);
  servoB.writeMicroseconds(b_min + theta45);
  servoC.writeMicroseconds(c_min + theta45);
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
unsigned int min_serial_elapsed = 5;
unsigned int last_serial_time = millis();

bool isMsg = false;


//________RASP_COMM_END_______//

void moveHead(float move_up, float move_right, float move_left) {

  // Debug output - print received values
  debug_print("HEAD DEBUG - Received: HF:");
  debug_print(move_up);
  debug_print(" HX:");
  debug_print(move_right);
  debug_print(" HY:");
  debug_println(move_left);

  //valori angoli iniziali
  a_0 = a_min + theta45;
  b_0 = b_min + theta45;
  c_0 = c_min + theta45;

  //altezza base
  // k= map(potValue_A, 0, 1024, -theta45, theta45);
  if (move_up == 0){
    k = 0;
  } else if (move_up == 335){
    k = 355;
  }else if (move_up == -335){
    k = -355;
  }else {
    k = 0;
    // Debug output when the value is not recognized
    debug_print("HF value not recognized: ");
    debug_println(move_up);
  }

  // Debug output - print k value
  debug_print("HEAD DEBUG - k value (vertical): ");
  debug_println(k);

  int slope = theta45 + abs(k);

  //inclinazione motori back a destra e sinistra
  b_bl = map(move_left, 0, 1024, slope, -slope);  //-45,45
  c_br = map(move_left, 0, 1024, -slope, +slope);

  //inclinazione motore avanti
  a_fb = map(move_right, 0, 1024, slope, -slope);

  forward_angle = map(move_right, 0, 1024, slope, -slope);
  b_fb = -forward_angle;
  c_fb = -forward_angle;

  a = constrain(a_0 + a_fb + k, a_min, a_max);
  b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
  c = constrain(c_0 + c_fb + c_br + k, c_min, c_max);

  servoA.writeMicroseconds(a);
  servoB.writeMicroseconds(b);
  servoC.writeMicroseconds(c);
}

void moveHeadBase(float move_forward, float move_angular) {
  const float amplitude = 335.0;
  const float maxFrequency_A = 45;
  const float minFrequency_B = 0.0;
  const float maxFrequency_B = 100;
  const float returnSpeed = 0.05;

  static unsigned long lastTime = 0;
  static float phase_A = 0.0;
  static float phase_BC = 0.0;
  static bool returningToInitial_A = false;
  static bool returningToInitial_BC = false;

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  static unsigned long phaseStartTime = 0;
  
  // Special case: If move_forward is exactly 42, trigger circular motion mode
  // (Using 42 as a special value that wouldn't normally be sent)
  if (move_forward == 42) {
    // Call the circular motion function
    moveHeadCircular();
    return;
  }

  if (move_forward != 0 && move_angular == 0 && move_forward != 42) {
    returningToInitial_A = false;
    returningToInitial_BC = true;

    unsigned long elapsedPhaseTime = currentTime - phaseStartTime;
    float frequency_A = maxFrequency_A;

    if (elapsedPhaseTime < 1000) {
      frequency_A = maxFrequency_A - 5;
    } else if (elapsedPhaseTime < 1400) {  //quando questo finisce, dev'essere più vicino possibile alla initial position-> regola il tempo in base a questo 
      frequency_A = 2 * maxFrequency_A ;
    } else if (elapsedPhaseTime < 2980) {  //quando questo finisce, dev'essere più vicino possibile alla initial position-> regola il tempo in base a questo 
      frequency_A = maxFrequency_A - 27;
    } else {
      phaseStartTime = currentTime;  // Restart phase timer
      lastTime = currentTime;  // Reset deltaTime to avoid inconsistencies
      return;
    }

    phase_A += 2 * PI * frequency_A / 100 * deltaTime;
    phase_A = fmod(phase_A, 2 * PI);

    a_fb = -amplitude * sin(phase_A);
    b_fb = -a_fb;
    c_br = -a_fb;

    a_0 = a_min + amplitude;
    b_0 = b_min + amplitude;
    c_0 = c_min + amplitude;

    a = constrain(a_0 + a_fb + k, a_min, a_max);
    b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
    c = constrain(c_0 + c_br + k, c_min, c_max);

    servoA.writeMicroseconds(a);
    servoB.writeMicroseconds(b);
    servoC.writeMicroseconds(c);
  }

  else if (move_angular != 0 && move_forward == 0) {
    returningToInitial_BC = false;
    returningToInitial_A = true;

    float absPotValue_B = abs(move_angular);
    float frequency_BC = minFrequency_B + (absPotValue_B / 1.1) * (maxFrequency_B - minFrequency_B);

    phase_BC += 2 * PI * frequency_BC / 100 * deltaTime;
    phase_BC = fmod(phase_BC, 2 * PI);

    if (move_angular > 0) {
      b_fb = b_min;
      c_br = -amplitude * sin(phase_BC);
    } else {
      b_fb = -amplitude * sin(phase_BC);
      c_br = c_min;
    }

    a_fb = 0;

    b_0 = b_min + amplitude;
    c_0 = c_min + amplitude;

    b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
    c = constrain(c_0 + c_br + k, c_min, c_max);

    servoA.writeMicroseconds(a_min + theta45);
    servoB.writeMicroseconds(b);
    servoC.writeMicroseconds(c);
  }

  if (move_forward == 0 && move_angular == 0) {
    if (!returningToInitial_A) {
      returningToInitial_A = true;
      phase_A = 0;
    }
    if (!returningToInitial_BC) {
      returningToInitial_BC = true;
      phase_BC = 0;
    }

    a = servoA.readMicroseconds();
    b = servoB.readMicroseconds();
    c = servoC.readMicroseconds();

    a = a + (a_min + theta45 - a) * returnSpeed;
    b = b + (b_min + theta45 - b) * returnSpeed;
    c = c + (c_min + theta45 - c) * returnSpeed;

    servoA.writeMicroseconds(a);
    servoB.writeMicroseconds(b);
    servoC.writeMicroseconds(c);
  }

  a_deg = map(a, 1220, 1890, 90, 0);
  b_deg = map(b, 1220, 1890, 90, 0);
  c_deg = map(c, 1220, 1890, 90, 0);
}

// New function to make servos move in a circular pattern
void moveHeadCircular() {
  const float amplitude = 335.0;  // Same amplitude as in other functions
  const float period = 3000;      // Time for a complete circle (in ms)
  const float phase_offset = 2 * PI / 3;  // 120 degrees offset between servos
  
  // Get current time
  unsigned long currentTime = millis();
  
  // Calculate phase based on time (0 to 2π)
  float phase = fmod((float)currentTime / period * 2 * PI, 2 * PI);
  
  // Calculate position for each servo with phase offset for circular motion
  // Using sine and cosine to create circular movement
  a_fb = -amplitude * sin(phase);
  b_fb = -amplitude * sin(phase + phase_offset);
  c_br = -amplitude * sin(phase + 2 * phase_offset);
  
  // Base angles (center position)
  a_0 = a_min + theta45;
  b_0 = b_min + theta45;
  c_0 = c_min + theta45;
  
  // Calculate and constrain final positions
  a = constrain(a_0 + a_fb, a_min, a_max);
  b = constrain(b_0 + b_fb, b_min, b_max);
  c = constrain(c_0 + c_br, c_min, c_max);
  
  // Write to servos
  servoA.writeMicroseconds(a);
  servoB.writeMicroseconds(b);
  servoC.writeMicroseconds(c);
  
  // Convert to degrees for debugging (if needed)
  a_deg = map(a, 1220, 1890, 90, 0);
  b_deg = map(b, 1220, 1890, 90, 0);
  c_deg = map(c, 1220, 1890, 90, 0);
}

void serial_loop() {
  now = millis();
  serial_elapsed = now - last_serial_time;

  if (serial_elapsed < min_serial_elapsed)
    return;

  // Legge eventuali comandi dalla seriale
  while (read_key_value_serial() && !isMsg) {
    isMsg = true;
  }

  // Se almeno un messaggio è stato ricevuto
  if (isMsg) {
    // Print received command array for debugging
    debug_print("Commands received: HF:");
    debug_print(lastSpeedCommand[0]);
    debug_print(" HX:");
    debug_print(lastSpeedCommand[1]);
    debug_print(" HY:");
    debug_print(lastSpeedCommand[2]);
    debug_print(" BF:");
    debug_print(lastSpeedCommand[3]);
    debug_print(" BB:");
    debug_println(lastSpeedCommand[4]);

    // Controlliamo se moveHeadBase è attivo
    bool moveHeadBaseActive = (lastSpeedCommand[3] != 0 || lastSpeedCommand[4] != 0);
    bool moveHeadActive = (lastSpeedCommand[0] != 0 || lastSpeedCommand[1] != 511 || lastSpeedCommand[2] != 511);
    
    // Special command for circular motion (using lastSpeedCommand[5])
    bool circularMotionActive = (lastSpeedCommand[5] == 1);

    // Print which command path we're taking
    if (circularMotionActive) {
      debug_println("Executing: Circular Motion");
      moveHeadCircular();
    }
    else if (moveHeadBaseActive) {
      debug_println("Executing: Head Base Motion");
      moveHeadBase(lastSpeedCommand[3], lastSpeedCommand[4]);
    }
    else if (moveHeadActive) {
      debug_println("Executing: Direct Head Control");
      moveHead(lastSpeedCommand[0], lastSpeedCommand[1], lastSpeedCommand[2]);
    }
    else {
      debug_println("Executing: Reset to Neutral");
      moveHeadBase(0, 0);  // Reset to neutral position
    }
  }

  last_serial_time = millis();
  isMsg = false;
}

void setup() {

  // 1 start serial and await a bit for warmup;
  Serial.begin(115200);
  Serial.flush();
  delay(200);

  servoA.attach(PIN_SERVO_A);
  servoB.attach(PIN_SERVO_B);
  servoC.attach(PIN_SERVO_C);

  canWrite = false;
  lastWriteTime = millis();

  // write_serial(ARDUINO_OK_FLAG);

  Serial.setTimeout(5000);

  prevcTime = millis();

  servoA.writeMicroseconds(a_min + theta45);
  servoB.writeMicroseconds(b_min + theta45);
  servoC.writeMicroseconds(c_min + theta45);

  debug_println("Arduino HEAD setup started");

  // Invia un messaggio quando il setup è completo
  debug_println("Setup HEAD Complete");
}

void loop() {
  watchdog_tick();
  serial_loop();
}

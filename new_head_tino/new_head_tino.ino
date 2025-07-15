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
    debug_println(lastSpeedCommand[2]);

    // Only use direct head control - HF (pitch), HX (pan), HY (tilt)
    bool moveHeadActive = (lastSpeedCommand[0] != 0 || lastSpeedCommand[1] != 511 || lastSpeedCommand[2] != 511);
    
    if (moveHeadActive) {
      debug_println("Executing: Direct Head Control");
      moveHead(lastSpeedCommand[0], lastSpeedCommand[1], lastSpeedCommand[2]);
    }
    else {
      debug_println("Executing: Reset to Neutral");
      // Reset to neutral position
      servoA.writeMicroseconds(a_min + theta45);
      servoB.writeMicroseconds(b_min + theta45);
      servoC.writeMicroseconds(c_min + theta45);
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

#include <Servo.h>
#include "RaspberryCommunication.h"

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
  }

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

// void moveHeadBase(float move_forward, float move_angular) {
//   // Constants for the oscillation
//   const float amplitude = 335.0;     // Fixed amplitude
//   const float minFrequency_A = 0.0;  // Minimum frequency when potValue is 0
//   const float maxFrequency_A = 50;   // Max frequency in Hz (adjust as needed)
//   const float minFrequency_B = 0.0;  // Minimum frequency when potValue is 0
//   const float maxFrequency_B = 100;  // Max frequency in Hz (adjust as needed)
//   const float returnSpeed = 0.05;    // Speed factor for gradual return to initial position (adjust as needed)

//   // Time tracking
//   static unsigned long lastTime = 0;
//   static float phase_A = 0.0;
//   static float phase_BC = 0.0;
//   static bool returningToInitial_A = false;
//   static bool returningToInitial_BC = false;

//   // Get current time
//   unsigned long currentTime = millis();

//   // Calculate the time difference since the last update (in seconds)
//   float deltaTime = (currentTime - lastTime) / 1000.0;
//   lastTime = currentTime;

//   // Case 1: Handle head oscillation (based on potValue_A, i.e., lastSpeedCommand[0])
//   if (move_forward != 0 && move_angular == 0) {  // Exclude potValue_B
//     returningToInitial_A = false;
//     returningToInitial_BC = true;  // Ensure BC returns to neutral

//     // Map potValue_A to frequency for A oscillation
//     float frequency_A = map(move_forward, 0, 25, minFrequency_A, maxFrequency_A);

//     // Increment the phase for A, B, and C
//     phase_A += 2 * PI * frequency_A / 100 * deltaTime;

//     // Keep phase in range
//     phase_A = fmod(phase_A, 2 * PI);

//     // Oscillate servo A
//     a_fb = - amplitude * sin(phase_A);

//     // Move B and C in synchronization with A
//     b_fb = -a_fb;
//     c_br = -a_fb;

//     // Base angles
//     a_0 = a_min + amplitude;
//     b_0 = b_min + amplitude;
//     c_0 = c_min + amplitude;

//     // Constrain positions for servos
//     a = constrain(a_0 + a_fb + k, a_min, a_max);
//     b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
//     c = constrain(c_0 + c_br + k, c_min, c_max);

//     // Write positions to servos
//     servoA.writeMicroseconds(a);
//     servoB.writeMicroseconds(b);
//     servoC.writeMicroseconds(c);
//   }

//   // Case 2: Handle BC oscillation (based on potValue_B, i.e., lastSpeedCommand[1])
//   else if (move_angular != 0 && move_forward == 0) {
//     returningToInitial_BC = false;
//     returningToInitial_A = true;  // Ensure A returns to neutral

//     // Map the absolute value of potValue_B to frequency for BC oscillation
//     float absPotValue_B = abs(move_angular);
//     float frequency_BC = minFrequency_B + (absPotValue_B / 1.1) * (maxFrequency_B - minFrequency_B);

//     // Increment the phase for BC oscillation
//     phase_BC += 2 * PI * frequency_BC / 100 * deltaTime;
//     phase_BC = fmod(phase_BC, 2 * PI);

//     // Oscillate servo B and C in opposite directions based on the sign of potValue_B
//     if (move_angular > 0) {
//       // b_fb = amplitude * sin(phase_BC);  // B goes in one direction
//       b_fb = b_min;
//       c_br = -amplitude * sin(phase_BC);  // C goes in the opposite direction
//     } else {
//       b_fb = -amplitude * sin(phase_BC);  // B goes in the opposite direction
//       // c_br = amplitude * sin(phase_BC);   // C goes in the other direction
//       c_br = c_min;
//     }

//     // Stop A
//     a_fb = 0;

//     // Base angles
//     b_0 = b_min + amplitude;
//     c_0 = c_min + amplitude;

//     // Constrain positions for servos
//     b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
//     c = constrain(c_0 + c_br + k, c_min, c_max);

//     // Write positions to servos
//     servoA.writeMicroseconds(a_min + theta45);  // A remains neutral
//     servoB.writeMicroseconds(b);
//     servoC.writeMicroseconds(c);
//   }

//   // Case 3: Handle return to neutral when both commands are zero
//   if (move_forward == 0 && move_angular == 0) {
//     if (!returningToInitial_A) {
//       returningToInitial_A = true;
//       phase_A = 0;  // Reset phase for A oscillation
//     }
//     if (!returningToInitial_BC) {
//       returningToInitial_BC = true;
//       phase_BC = 0;  // Reset phase for BC oscillation
//     }

//     // Gradual return to initial positions for all servos
//     a = servoA.readMicroseconds();
//     b = servoB.readMicroseconds();
//     c = servoC.readMicroseconds();

//     a = a + (a_min + theta45 - a) * returnSpeed;
//     b = b + (b_min + theta45 - b) * returnSpeed;
//     c = c + (c_min + theta45 - c) * returnSpeed;

//     servoA.writeMicroseconds(a);
//     servoB.writeMicroseconds(b);
//     servoC.writeMicroseconds(c);
//   }

//   // Debugging: Convert values to degrees and print
//   a_deg = map(a, 1220, 1890, 90, 0);
//   b_deg = map(b, 1220, 1890, 90, 0);
//   c_deg = map(c, 1220, 1890, 90, 0);

//   // Serial.print(" ThetaA: ");
//   // Serial.print(a_deg);
//   // Serial.print(", ThetaB: ");
//   // Serial.print(b_deg);
//   // Serial.print(", ThetaC: ");
//   // Serial.println(c_deg);
// }

// void moveHeadBase(float move_forward, float move_angular) {
//   const float amplitude = 335.0;
//   const float maxFrequency_A = 46;
//   const float minFrequency_B = 0.0;
//   const float maxFrequency_B = 100;
//   const float returnSpeed = 0.05;

//   static unsigned long lastTime = 0;
//   static float phase_A = 0.0;
//   static float phase_BC = 0.0;
//   static bool returningToInitial_A = false;
//   static bool returningToInitial_BC = false;

//   unsigned long currentTime = millis();
//   float deltaTime = (currentTime - lastTime) / 1000.0;
//   lastTime = currentTime;

//   static unsigned long phaseStartTime = 0;

//   if (move_forward != 0 && move_angular == 0) {
//     returningToInitial_A = false;
//     returningToInitial_BC = true;

//     unsigned long elapsedPhaseTime = currentTime - phaseStartTime;
//     float frequency_A = maxFrequency_A;

//     if (elapsedPhaseTime < 1100) {
//       frequency_A = maxFrequency_A;
//     } else if (elapsedPhaseTime < 1500) {
//       frequency_A = 2 * maxFrequency_A + 20;
//     }
//       else if (elapsedPhaseTime < 3200) {
//       frequency_A =  maxFrequency_A;
//     } else {
//       phaseStartTime = currentTime;
//     }

//     phase_A += 2 * PI * frequency_A / 100 * deltaTime;
//     phase_A = fmod(phase_A, 2 * PI);

//     a_fb = -amplitude * sin(phase_A);
//     b_fb = -a_fb;
//     c_br = -a_fb;

//     a_0 = a_min + amplitude;
//     b_0 = b_min + amplitude;
//     c_0 = c_min + amplitude;

//     a = constrain(a_0 + a_fb + k, a_min, a_max);
//     b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
//     c = constrain(c_0 + c_br + k, c_min, c_max);

//     servoA.writeMicroseconds(a);
//     servoB.writeMicroseconds(b);
//     servoC.writeMicroseconds(c);
//   }

//   else if (move_angular != 0 && move_forward == 0) {
//     returningToInitial_BC = false;
//     returningToInitial_A = true;

//     float absPotValue_B = abs(move_angular);
//     float frequency_BC = minFrequency_B + (absPotValue_B / 1.1) * (maxFrequency_B - minFrequency_B);

//     phase_BC += 2 * PI * frequency_BC / 100 * deltaTime;
//     phase_BC = fmod(phase_BC, 2 * PI);

//     if (move_angular > 0) {
//       b_fb = b_min;
//       c_br = -amplitude * sin(phase_BC);
//     } else {
//       b_fb = -amplitude * sin(phase_BC);
//       c_br = c_min;
//     }

//     a_fb = 0;

//     b_0 = b_min + amplitude;
//     c_0 = c_min + amplitude;

//     b = constrain(b_0 + b_fb + b_bl + k, b_min, b_max);
//     c = constrain(c_0 + c_br + k, c_min, c_max);

//     servoA.writeMicroseconds(a_min + theta45);
//     servoB.writeMicroseconds(b);
//     servoC.writeMicroseconds(c);
//   }

//   if (move_forward == 0 && move_angular == 0) {
//     if (!returningToInitial_A) {
//       returningToInitial_A = true;
//       phase_A = 0;
//     }
//     if (!returningToInitial_BC) {
//       returningToInitial_BC = true;
//       phase_BC = 0;
//     }

//     a = servoA.readMicroseconds();
//     b = servoB.readMicroseconds();
//     c = servoC.readMicroseconds();

//     a = a + (a_min + theta45 - a) * returnSpeed;
//     b = b + (b_min + theta45 - b) * returnSpeed;
//     c = c + (c_min + theta45 - c) * returnSpeed;

//     servoA.writeMicroseconds(a);
//     servoB.writeMicroseconds(b);
//     servoC.writeMicroseconds(c);
//   }

//   a_deg = map(a, 1220, 1890, 90, 0);
//   b_deg = map(b, 1220, 1890, 90, 0);
//   c_deg = map(c, 1220, 1890, 90, 0);
// }

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

  if (move_forward != 0 && move_angular == 0) {
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
      // Bring servos back to the initial position
      // servoA.writeMicroseconds(a_min + theta45);
      // servoB.writeMicroseconds(b_min + theta45);
      // servoC.writeMicroseconds(c_min + theta45);
      // phase_A = 0;  // Reset phase
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
    // Serial.print("HUP: ");
    // Serial.println(lastSpeedCommand[0]);
    // Controlliamo se moveHeadBase è attivo
    bool moveHeadBaseActive = (lastSpeedCommand[3] != 0 || lastSpeedCommand[4] != 0);
    bool moveHeadActive = (lastSpeedCommand[0] != 0 || lastSpeedCommand[1] != 511 || lastSpeedCommand[2] != 511);

    // Se moveHeadBase è attivo, eseguiamo solo moveHeadBase
    if (moveHeadBaseActive) {
      // Serial.println("Executing moveHeadBase...");
      moveHeadBase(lastSpeedCommand[3], lastSpeedCommand[4]);
    }
    // Se moveHeadBase non è attivo, eseguiamo moveHead se i comandi sono validi
    else if (moveHeadActive) {
      // Serial.println("Executing moveHead...");
      moveHead(lastSpeedCommand[0], lastSpeedCommand[1], lastSpeedCommand[2]);
    }
    
    // Reset quando entrambi i comandi sono neutrali
    if (!moveHeadBaseActive && !moveHeadActive) {
      // Serial.println("No command received...");
      moveHeadBase(0, 0);  // Reset della base
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

  Serial.println("Arduino HEAD setup started");

  // Invia un messaggio quando il setup è completo
  Serial.println("Setup HEAD Complete");
}

void loop() {
  watchdog_tick();
  serial_loop();
}

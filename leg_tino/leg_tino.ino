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
// Configurazione del driver motore
CytronMD motor(PWM_DIR, _2A, _2B);  // Motore

int state = 0;

//______________PID
float vt = 0;
float SampleTime = 20.0;
int outMax = 200;
int outMin = -200;
volatile int ticks = 0;
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
//_______________
bool isSinusoidalActive = false;
unsigned long startTime = millis();  // Momento iniziale del ciclo sinusoidale
static float cycleTime = 3.1;        // Tempo totale del ciclo in secondi, valore iniziale di default
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
  } else {
    ticks--;
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
  button.loop();  // Aggiungi una seconda chiamata per leggere il bottone più frequentemente
  state = button.getState();
  static float lastVt = 0;  // Store the last value of vt

  if (linearX != 0 && angular == 0) {
    if (!isSinusoidalActive) {
      // Attiva il profilo sinusoidale e resetta startTime
      isSinusoidalActive = true;
      startTime = millis();  // Resetta il tempo di inizio del ciclo
    }
    // Calcola vt usando il profilo sinusoidale
    // vt = vtSinusoidal();
    vt = constrain(vtSinusoidal(), 0, 180);  // Limita Vt a un massimo di 180
    lastVt = vt;
    // Memorizza l'ultimo valore di vt

    // button.loop();  // Aggiungi una seconda chiamata per leggere il bottone più frequentemente
    // state = button.getState();

    // Resetta il ciclo se il pulsante è premuto
    if (isPressed) {
      Serial.print("elapsedTime:  ");
      float elapsedTime = (millis() - startTime) / 1000.0;
      Serial.println(elapsedTime);
      cycleTime = constrain(elapsedTime, 3.0, 4.0);  // Limita la durata del nuovo ciclo tra 3 e 5 secondi
      startTime = millis();                          // Resetta il tempo di inizio del ciclo
    }
  } else if (linearX == 0 && angular == 0) {
    if (isSinusoidalActive) {
      isSinusoidalActive = false;  // Disattiva il profilo sinusoidale
    }

    if (state == HIGH) {
      vt = 90;  // Quando linearX == 0, ritorna alla posizione iniziale con velocità controllata
    } else if (state == LOW) {
      vt = 0;  // Ferma il motore quando il bottone è premuto
    }
  } else if (linearX != 0 && angular != 0) {
    // Se linearX e angular sono entrambi diversi da zero, fermati o torna alla posizione iniziale
    if (state == HIGH) {
      vt = 120;  // Vai alla posizione iniziale finché il bottone non viene premuto
    } else if (state == LOW) {
      vt = 0;  // Ferma il motore
    }
  }
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

  // Serial.print("Vt:");
  // Serial.println(vt);
  // Serial.print(".");
  // Serial.print("rpmFilt:");
  // Serial.println(rpmFilt);

  int pwm = (int)(u);
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
    // Serial.print(lastSpeedCommand[0]);
    canWrite = true;
    last_serial_time = millis();
    isMsg = false;
  }
  moveLeg(lastSpeedCommand[0], lastSpeedCommand[1]);
  // Serial.println(lastSpeedCommand[0]);
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
  }

  canWrite = false;
  lastWriteTime = millis();
  write_serial(LEG_OK_FLAG);
  Serial.setTimeout(5000);
  prevTime = millis();

  moveLeg(0, 0);

  Serial.println("Arduino LEG setup started");
  Serial.println("Setup LEG Complete");
}

void loop() {

  button.loop();
  state = button.getState();  // state == HIGH --> UNTOUCHED == 1 || state == LOW --> TOUCHED == 0
                              //codice giusto

  isPressed = ButtonisPressed();

  serial_loop();

  computeOutput();
}
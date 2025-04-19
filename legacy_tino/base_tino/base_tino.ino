#include <CytronMotorDriver.h>
#include <Encoder.h>
#include <ViRHas.h>

#include "RaspberryCommunication.h"
#include "pinout.h"

//_________________CYTRON_MOTOR___________________________//
//Maximum speed wantd
#define _MAX_SPEED 20       //cm/s
#define _MAX_ANGULAR 0.6    // rad/s
#define wheel_radius 3.0f   //cm
#define robot_radius 16.5f  //cm
#define encoder_ppr 1920.0f
//zigler-nichols PID Tuning
#define Ku 21
#define Tu 2.25

float Kp = 7.3;
float Ki = 5.6;
float Kd = 0.2;

// Configure the motor driver.
CytronMD motor1(PWM_PWM, _2_1A, _2_1B);  //Motor 1 : Atras
CytronMD motor2(PWM_PWM, _1_1A, _1_1B);  //Motor 2 : right robot
CytronMD motor3(PWM_PWM, _1_2A, _1_2B);  //Motor 3 : left robot

//enable the encoders and set each eancoder of each sensor to which pin is connectes. take into coount that the
//order has to be Motor1,Motor2,Motor3. The order used in the motor has to joing with this one
Encoder ENCODER[] = { Encoder(_EP11, _EP12), Encoder(_EP21, _EP22), Encoder(_EP31, _EP32) };

//robot class
ViRHaS virhas = ViRHaS(motor1, motor2, motor3, ENCODER[0], ENCODER[1], ENCODER[2]);


char debug_msg_static[100];

//_______________________________
unsigned long phaseStartTimeLinear = 0;  // Timer per il movimento lineare
unsigned long phaseStartTimeAngular = 0; // Timer per il movimento angolare
bool wasLinearYActive = false;     // Tracks if linearY was previously active
bool wasAngularActive = false;     // Tracks if angular was previously active
bool angularDirectionPositive = false; // Track the direction of angular movement
//_______________________________

float convertSpeed(float speed, bool angular) {
  if (angular) {
    if (speed > _MAX_ANGULAR)
      return _MAX_ANGULAR;
    if (speed < -_MAX_ANGULAR)
      return -_MAX_ANGULAR;
    else
      return speed;
  }
  if (speed > _MAX_SPEED)
    return _MAX_SPEED;
  if (speed < -_MAX_SPEED)
    return -_MAX_SPEED;
  else
    return speed;
}

void moveRobot(float linearY, float linearX, float angular) {
  if (linearX == 0.00 && linearY == 0.00 && angular == 0.00) {
    virhas.stop();
  } else {
    virhas.run2(convertSpeed(-linearY, false), convertSpeed(linearX, false), convertSpeed(-angular, true));
    virhas.PIDLoop(debug_msg_static);
  }
}

void updateBaseMovementByTime(float linearY, float linearX, float angular) {
  unsigned long currentTime = millis();
  unsigned long elapsedTimeLinear = currentTime - phaseStartTimeLinear;
  unsigned long elapsedTimeAngular = currentTime - phaseStartTimeAngular;

  // Gestione transizioni
  if (linearY != 0 && !wasLinearYActive && angular == 0) {
    phaseStartTimeLinear = millis();  // Reset del timer per linearY quando diventa attivo
    wasLinearYActive = true;
    wasAngularActive = false;
  } else if (angular != 0 && !wasAngularActive && linearY == 0) {
    phaseStartTimeAngular = millis();  // Reset del timer per angular quando diventa attivo
    wasAngularActive = true;
    wasLinearYActive = false;
    angularDirectionPositive = angular > 0; // Memorizza la direzione del movimento angolare
  } else if (linearY == 0 && angular == 0) {
    wasLinearYActive = false;
    wasAngularActive = false;
  }

  if (linearY != 0 && angular == 0) {
    // Profilo di movimento lineare
    if (elapsedTimeLinear < 1300) {
      linearY = 0;  // Prima fase: 2 secondi di pausa
    } else if (elapsedTimeLinear >= 1300 && elapsedTimeLinear < 3020) {
      linearY = 16;  // Seconda fase: 1 secondo di movimento in avanti
    } else {
      phaseStartTimeLinear = millis();  // Reset del ciclo dopo 3 secondi totali
    }

    moveRobot(linearY, linearX, angular);

  } else if (angular != 0 && linearY == 0) {
    // Profilo di movimento angolare
    if (angularDirectionPositive) {
      if (elapsedTimeAngular < 500) {
        angular = 0.5;  // Prima fase: 500 ms a 0.5 rad/s
      } else if (elapsedTimeAngular >= 500 && elapsedTimeAngular < 1000) {
        angular = 0;  // Seconda fase: 500 ms a 0 rad/s
      } else {
        phaseStartTimeAngular = millis();  // Reset del ciclo dopo 1 secondo totale
      }
    } else {
      if (elapsedTimeAngular < 500) {
        angular = -0.5;  // Prima fase: 500 ms a -0.5 rad/s
      } else if (elapsedTimeAngular >= 500 && elapsedTimeAngular < 1000) {
        angular = 0;  // Seconda fase: 500 ms a 0 rad/s
      } else {
        phaseStartTimeAngular = millis();  // Reset del ciclo dopo 1 secondo totale
      }
    }

    moveRobot(linearY, linearX, angular);

  } else if (linearY != 0 && angular != 0) {
    // Gestione della situazione intermedia: se entrambi sono diversi da zero fin dall'inizio
    if (!wasLinearYActive && !wasAngularActive) {
      linearY = 0;
      angular = 0;
    } else if (wasLinearYActive && !wasAngularActive) {
      angular = 0;  // Se linearY era attivo e angular diventa diverso da zero, continua con linearY
    } else if (wasAngularActive && !wasLinearYActive) {
      linearY = 0;  // Se angular era attivo e linearY diventa diverso da zero, continua con angular
    }
    moveRobot(linearY, linearX, angular);
  } else {
    moveRobot(linearY, linearX, angular);
  }
}

//___________________END_CYTRON_MOTOR________________________//

#define MAX_WATCHDOG_ELAPSED_TIME 100000

void reset_all_target_speeds() {
  moveRobot(0, 0, 0);
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

//Set values to motors
void serial_loop() {
  // a serial loop can only be performed once every SERIAL_ELAPSED time
  now = millis();

  serial_elapsed = now - last_serial_time;
  if (serial_elapsed < min_serial_elapsed)
    return;

  // read everything it can from serial
  while (read_key_value_serial() && !isMsg) {
    isMsg = true;
  }

  // if at least a msg was received, use the updated values
  if (isMsg) {

    updateBaseMovementByTime(lastSpeedCommand[0], 0.00, lastSpeedCommand[1]);

    canWrite = true;

    last_serial_time = millis();

    isMsg = false;
  }
}

void setup() {

  virhas.setKpid(Kp, Ki, Kd);
  virhas.setWheelRadius(wheel_radius);
  virhas.setEncoderPPR(encoder_ppr);
  virhas.setRobotRadius(robot_radius);
  virhas.stop();

  // 1 start serial and await a bit for warmup;
  Serial.begin(115200);
  Serial.flush();
  delay(200);

  canWrite = false;
  lastWriteTime = millis();

  // write_serial(ARDUINO_OK_FLAG);

  Serial.setTimeout(5000);

  prevcTime = millis();

  Serial.println("Arduino BASE setup started");

  // Invia un messaggio quando il setup è completo
  Serial.println("Setup BASE Complete");
}

void loop() {

  watchdog_tick();
  serial_loop();
}

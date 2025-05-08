// MOTORS 
// encoder pins - corrected based on testing
#define _EP21 19 // white right motor encoder
#define _EP22 18 // yellow right motor encoder
#define _EP31 21 // white left motor encoder
#define _EP32 20 // yellow left motor encoder

// driver pins - verified with testing (kept original pin numbers)
#define _1_1A 8  // PWM pin for left motor (was labeled as right in original code)
#define _1_1B 9  // DIR pin for left motor
#define _1_2A 10 // PWM pin for right motor (was labeled as left in original code)
#define _1_2B 11 // DIR pin for right motor

// Serial communication timing constants
#define MAX_WATCHDOG_ELAPSED_TIME 100000 // ms
#define MIN_SERIAL_ELAPSED 5 // ms
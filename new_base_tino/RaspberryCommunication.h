#ifndef _RASPBERRY_COMMUNICATION_H_
#define _RASPBERRY_COMMUNICATION_H_

#include <Arduino.h>

// Define communication constants
#define BUFFER_SIZE 150
#define COMMAND_TIMEOUT 2000 // ms

// Command markers and parsing characters
#define KEY_BF "BF:"
#define KEY_BB "BB:"
#define KEY_HP "HP:"
#define KEY_HX "HX:"
#define KEY_HY "HY:"
#define SEP "_"

// Function declarations
void parse_key_value(char* key, float* value, char* buffer);
bool is_key(char* key, char* buffer);
bool read_key_value_serial();
void write_serial(const char* msg);
void process_special_command(char* buffer); // Added declaration for process_special_command

// Store the last speed command from Jetson
extern float lastSpeedCommand[2]; // [0] = linear velocity, [1] = angular velocity
extern unsigned long last_command_time; // For watchdog
extern unsigned long current_time; // For watchdog

// External buffer declaration
extern char buffer[BUFFER_SIZE];

#endif // _RASPBERRY_COMMUNICATION_H_
// RaspberryCommunication.h
#ifndef RASPBERRY_COMMUNICATION_H
#define RASPBERRY_COMMUNICATION_H

#include <Arduino.h>

// Definizione delle costanti
#define EMPTY_STRING " "
#define DELIMITER ":"
#define MSG_DELIMITER "_"
#define LEG_OK_FLAG "OK"
#define REQUEST_ARDUINO_RESET "RESET"
#define ARDUINO_READY_FLAG "READY"
#define RASP_DEFAULT "A"

// Chiavi dei messaggi ricevuti
#define BASE_FORWARD_KEY "BF"
#define BASE_ANGULAR_KEY "BB"

// Configurazioni di setup
#define MAX_READ_MSGS 5  // numero massimo di messaggi letti per iterazione


struct KeyValueMsg {
  String key;
  String value;  
};

extern String current_data;
extern bool dirty;
extern KeyValueMsg current_msg;
extern unsigned long last_command_time;
extern unsigned long current_time;
extern float lastSpeedCommand[3];

void write_serial(String msg);
void message_response(KeyValueMsg keyValueMsg);
void get_key_value(String msg);
void split_msg(String msg);
bool read_serial();
bool read_key_value_serial();
void watchdog_tick();

#endif // RASPBERRY_COMMUNICATION_H

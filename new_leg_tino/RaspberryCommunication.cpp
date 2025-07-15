// RaspberryCommunication.cpp
#include "RaspberryCommunication.h"

// Global variables
String current_data = EMPTY_STRING;
bool dirty;
KeyValueMsg current_msg = {"", ""}; // global variable with the current received message
unsigned long last_command_time;
unsigned long current_time;
float lastSpeedCommand[2] = {0,0};
float lastSpeedCommandPrev = 0;  // Variabile per memorizzare l'ultimo comando di velocità ricevuto


void write_serial(String msg) {
  //Serial.println(msg);
}

void message_response(KeyValueMsg keyValueMsg){
  // act based on a message - discriminate using the key

  String key = keyValueMsg.key;
  String value = keyValueMsg.value;

  if(key == BASE_FORWARD_KEY) {
    lastSpeedCommand[0] = value.toFloat();
    dirty = true;
    last_command_time = millis();
    }
  else if (key == BASE_ANGULAR_KEY) {
    lastSpeedCommand[1] = value.toFloat();
    dirty = true;
    last_command_time = millis();
    }
  else {
    // write_serial("[ERROR] unsupported message key: " + key);
  }
}

void get_key_value(String msg) {
  // 1 find the position of the delimiter
  // 2 get the first and second substrings: the KEY and VALUE of the message

  int delim_index = msg.indexOf(DELIMITER);

  String key = msg.substring(0, delim_index);
  String value = msg.substring(delim_index + 1);

  KeyValueMsg tempKeyValueMsg = KeyValueMsg();
  tempKeyValueMsg.key = key;
  tempKeyValueMsg.value = value;

  message_response(tempKeyValueMsg);
}

void split_msg(String msg) {
  if (current_data == RASP_DEFAULT)
    return;

  int startDelimiterIndex = -1;
  int endDelimiterIndex = msg.indexOf(MSG_DELIMITER);

  if (endDelimiterIndex == -1) {
    get_key_value(msg);
    return;
  }

  bool firstRound = true;

  while (true) {
    if (firstRound) {
      startDelimiterIndex = -1;
      endDelimiterIndex = msg.indexOf(MSG_DELIMITER);
      firstRound = false;
    } else {
      startDelimiterIndex = endDelimiterIndex;
      endDelimiterIndex = msg.indexOf(MSG_DELIMITER, startDelimiterIndex + 1);
    }
    if (endDelimiterIndex == -1) {
      get_key_value(msg.substring(startDelimiterIndex + 1));
      break;
    }
    get_key_value(msg.substring(startDelimiterIndex + 1, endDelimiterIndex));
  }
}

bool read_serial() {
  //Serial.println("Reading from serial...");
  if (Serial.available() > 0) {
    //Serial.println("Data available to read.");
    current_data = Serial.readStringUntil('\n');
    //Serial.println("Data read");
    return true;
  }
  else {
    //Serial.println("No data available to read.");
    return false;
  }
}

bool read_key_value_serial(){
  //Serial.println("Reading key-value pair from serial...");
  if (read_serial()){
    split_msg(current_data);
    return true;
  }
  else {
    return false;
  }
}


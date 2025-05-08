#include "RaspberryCommunication.h"

// Global variables
float lastSpeedCommand[2] = {0.0, 0.0}; // [0] = linear velocity, [1] = angular velocity
unsigned long last_command_time = 0;
unsigned long current_time = 0;

// Buffer for serial communication
char buffer[BUFFER_SIZE];
int buffer_index = 0;

// Parse a key-value pair from buffer
void parse_key_value(char* key, float* value, char* buffer) {
  int key_length = strlen(key);
  
  // Find key in buffer
  char* key_start = strstr(buffer, key);
  if (key_start != NULL) {
    // Extract the value after the key
    char tmp[20];
    int i = 0;
    key_start += key_length; // Move pointer to after the key
    
    // Copy value until separator or end of string
    while (*key_start != '\0' && *key_start != '_' && i < 19) {
      tmp[i++] = *key_start++;
    }
    tmp[i] = '\0';
    
    // Convert string to float
    *value = atof(tmp);
  }
}

// Check if a key exists in the buffer
bool is_key(char* key, char* buffer) {
  return (strstr(buffer, key) != NULL);
}

// Read and process a command from serial
bool read_key_value_serial() {
  bool dataReceived = false;
  
  // Check if there's data available
  if (Serial.available() > 0) {
    // Read a line
    String receivedData = Serial.readStringUntil('\n');
    
    // Copy to buffer with bounds checking
    receivedData.toCharArray(buffer, BUFFER_SIZE);
    
    // Parse linear velocity (BF = Base Forward)
    if (is_key(KEY_BF, buffer)) {
      parse_key_value(KEY_BF, &lastSpeedCommand[0], buffer);
    }
    
    // Parse angular velocity (BB = Base rotation)
    if (is_key(KEY_BB, buffer)) {
      parse_key_value(KEY_BB, &lastSpeedCommand[1], buffer);
    }
    
    // Update command timestamp for watchdog
    last_command_time = millis();
    dataReceived = true;
  }
  
  return dataReceived;
}

// Send a message back to Jetson
void write_serial(const char* msg) {
  Serial.println(msg);
}
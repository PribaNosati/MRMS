#include <ESP32CANBus.h> // Common libary for all CAN Bus devices
#include <mrm-node.h> // CAN Library for mrm-ref-can

ESP32CANBus canBus; // Instance of common library
Mrm_8x8a mrm_8x8a(&canBus); // Instance of library for mrm-ref-can

#define CAN_ID_LED8x8_IN 0x0200 // Inbound CAN Bus id
#define CAN_ID_LED8x8_OUT 0x0201 // Outbound CAN Bus id

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_8x8a.add(CAN_ID_LED8x8_IN, CAN_ID_LED8x8_OUT); // Add one mrm-8x8a board
  mrm_8x8a.test();  // Start testing
}

void loop() {
}

// If anything goes wrong, display the source and stop.
void error(String message){
  printf("%s", message);
  while(1);
}
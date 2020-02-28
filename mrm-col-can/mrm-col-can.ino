#include <ESP32CANBus.h> // Common libary for all CAN Bus devices
#include <mrm-node.h> // CAN Library for mrm-ref-can

ESP32CANBus canBus; // Instance of common library
Mrm_node mrm_node(&canBus); // Instance of library for mrm-ref-can

#define CAN_ID_NODE_IN 0x0170 // Inbound CAN Bus id
#define CAN_ID_NODE_OUT 0x0171 // Outbound CAN Bus id

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_node.add(CAN_ID_NODE_IN, CAN_ID_NODE_OUT); // Add one mrm-node board
  mrm_node.test();  // Start testing
}

void loop() {
}

// If anything goes wrong, display the source and stop.
void error(String message){
  printf("%s", message);
  while(1);
}
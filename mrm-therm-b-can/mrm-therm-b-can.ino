#include <ESP32CANBus.h> // Common libary for all CAN Bus devices
#include <mrm-therm-b-can.h> // CAN Library for mrm-ref-can

ESP32CANBus canBus; // Instance of common library
Mrm_therm_b_can mrm_therm_b_can(&canBus); // Instance of library for mrm-ref-can

#define CAN_ID_THERM_IN 0x0210 // Inbound CAN Bus id
#define CAN_ID_THERM_OUT 0x0211 // Outbound CAN Bus id

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_therm_b_can.add(CAN_ID_THERM_IN, CAN_ID_THERM_OUT); // Add one mrm-therm-b-can board
  mrm_therm_b_can.test();  // Start testing
}

void loop() {
}

// If anything goes wrong, display the source and stop.
void error(String message){
  printf("%s", message);
  while(1);
}
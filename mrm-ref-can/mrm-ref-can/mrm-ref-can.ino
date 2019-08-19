#include <ESP32CANBus.h> // Common libary for all CAN Bus devices
#include <mrm-ref-can_esp32.h> // CAN Library for mrm-ref-can

ESP32CANBus canBus; // Instance of common library
Mrm_ref_can mrm_ref_can(&canBus); // Instance of library for mrm-ref-can

#define CAN_ID_REF_CAN_IN 0x0160 // Inbound CAN Bus id
#define CAN_ID_REF_CAN_OUT 0x0161 // Outbound CAN Bus id

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_ref_can.add(CAN_ID_REF_CAN_IN, CAN_ID_REF_CAN_OUT); // Add one CAN Bus reflectance sensor array
  mrm_ref_can.test();  // Start testing
}

void loop() {
}

// If anything goes wrong, display the source and stop.
void error(String message){
  printf("%s", message);
  while(1);
}
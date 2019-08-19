#include <ESP32CANBus.h> // Common libary for all CAN Bus devices
#include <mrm-lid-can-b.h> // CAN Library for mrm-lid-can-b

ESP32CANBus canBus; // Instance of common library
Mrm_lid_can_b mrm_lid_can_b(&canBus);// Instance of library for mrm-lid-can-b

#define CAN_ID_VL531X_IN 0x0150 // Inbound CAN Bus id
#define CAN_ID_VL531X_OUT 0x0151 // Outbound CAN Bus id

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_lid_can_b.add(CAN_ID_VL531X_IN, CAN_ID_VL531X_OUT); // Add one CAN Bus lidar
  mrm_lid_can_b.test();  // Start testing
}

void loop() {
}

// If anything goes wrong, display the source and stop.
void error(String message){
  printf("%s", message);
  while(1);
}

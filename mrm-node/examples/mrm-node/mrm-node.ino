#include <mrm-node.h>

Mrm_node mrm_node; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_node.add(); // Add one mrm-node board
  mrm_node.test();  // Start testing
}

void loop() {
}
#include <mrm-ir-finder-can.h>

Mrm_ir_finder_can mrm_ir_finder_can; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_ir_finder_can.add(); // Add one mrm-ir-finder-can board
  mrm_ir_finder_can.test();  // Start testing
}

void loop() {
}
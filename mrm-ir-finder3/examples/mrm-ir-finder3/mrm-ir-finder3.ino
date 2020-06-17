#include <mrm-ir-finder3.h>

Mrm_ir_finder3 mrm_ir_finder3; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_ir_finder3.add(); // Add one mrm-ir-finder-can board
  mrm_ir_finder3.test();  // Start testing
}

void loop() {
}

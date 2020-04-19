#include <mrm-ref-can.h>

Mrm_ref_can mrm_ref_can; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_ref_can.add(); // Add one mrm-col-can board
  mrm_ref_can.test();  // Start testing
}

void loop() {
}
#include <mrm-lid-can-b2.h>

Mrm_lid_can_b2 mrm_lid_can_b2; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_lid_can_b2.add(); // Add one mrm-lid-can-b2 board
  mrm_lid_can_b2.test();  // Start testing
}

void loop() {
}
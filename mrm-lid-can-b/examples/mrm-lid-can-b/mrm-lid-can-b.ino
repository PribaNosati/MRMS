#include <mrm-lid-can-b.h>

Mrm_lid_can_b mrm_lid_can; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_lid_can.add(); // Add one mrm-lid-can-b board
  mrm_lid_can.test();  // Start testing
}

void loop() {
}
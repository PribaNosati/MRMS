#include <mrm-therm-b-can.h>

Mrm_therm_b_can mrm_therm_b_can; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_therm_b_can.add(); // Add one mrm-therm-b-can board
  mrm_therm_b_can.test();  // Start testing
}

void loop() {
}
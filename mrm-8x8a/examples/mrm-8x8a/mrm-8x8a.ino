#include <mrm-8x8a.h>

Mrm_8x8a mrm_8x8a; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_8x8a.add(); // Add one mrm-8x8a board
  mrm_8x8a.test();  // Start testing
}

void loop() {
}
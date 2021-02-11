#include <mrm-col-can.h>

Mrm_col_can mrm_col_can; // Instance of library for mrm-ref-can

void setup() {
  Serial.begin(115200); // Start serial communication with PC at 115000 bits per second
  delay(500); // Delay to allow serial port to start
  Serial.println("Start");

  mrm_col_can.add(); // Add one mrm-col-can board
  mrm_col_can.start();
}

void loop() {
    mrm_col_can.test(1); 
}

#include "IRReceivers.h"

IRReceivers ir; //Define the object

void setup() {
  //Add sensors. 
  ir.add(2, 60); //Pin 2, 60 degrees
  ir.add(3, 120); 
  ir.add(4, -60);
  
  Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable
  delay(500);
  Serial.println("Start");

  ir.test(); // Study the test() function in order to use it in Your code.
}

void loop() {}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(String message) {
  Serial.print(message);
  while (true)
    ;
}

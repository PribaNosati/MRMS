#include "IRFinders.h"

IRFinders ir; //Define the object

void setup() {
  //Add a sensor. 
  ir.add(A7, A10); //Analog input pin (A7), connected to sensor's DEG output (angle), A10 to CM (distance).
  
  Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable
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
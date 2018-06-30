#include "UltrasonicAsyncPWM.h"

UltrasonicAsyncPWM us; //Define the object

void setup() {
  //Add sensors. Echo pins (2. argument) must support interrupts. For Arduino Nano 2 and 3 are the only choices. For Teensy 3.2 You can use any.
  us.add(4, 2); // Trigger digital pin 4, echo pin 2
  us.add(5, 3); // Trigger digital pin 5, echo pin 3
  
  Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable
  delay(500);
  Serial.println("Start");

  us.test(); // Study the test() function in order to use it in Your code.
}

void loop() {}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(const char * message) {
  Serial.print(message);
  while (true)
    ;
}

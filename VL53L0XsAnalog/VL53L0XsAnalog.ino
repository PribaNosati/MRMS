#include "VL53L0XsAnalog.h"

VL53L0XsAnalog lidar; // Lidar object

void setup()
{
	Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable

	lidar.add(A10); // Analog input pin (A10), connected to sensor's AN output
	lidar.test(); // Run endless test.
}

void loop() {}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(String message) {
	Serial.print(message);
	while (true)
		;
}

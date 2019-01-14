#include "IMUBoschBNO055Analog.h"

IMUBoschBNO055Analog imu; // IMU object

void setup()
{
	Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable

	imu.add(A10); // Analog input pin (A10), connected to IMU's AN output. Uncomment for mrm-umu-an, comment for mrm-imu-an3.
	//imu.add(A10, A9, A8); // Analog input pins (A10, A9, and A8), connected to IMU's YAW, PIT, and ROLL outputs. Uncomment for mrm-umu-an3, comment for mrm-imu-an.
	imu.test(); // Run endless test.
}

void loop() {}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(String message) {
	Serial.print(message);
	while (true)
		;
}
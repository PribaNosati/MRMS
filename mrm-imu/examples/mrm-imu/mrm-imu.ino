#include <Wire.h>
#include <mrm-imu.h>


Mrm_imu *imu;

void setup()
{
	Serial.begin(115200);// Start communication with a computer connected to Arduino via a USB cable
	Wire.begin(); // Start I2C

	imu = new Mrm_imu();
	imu->add(true); 
}

void loop(){
  imu->test();
}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(String message) {
	Serial.print(message);
	while (true)
		;
}

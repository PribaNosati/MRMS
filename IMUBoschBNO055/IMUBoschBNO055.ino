#include <i2c_t3.h>
#include <IMUBoschBNO055.h>

IMUBoschBNO055 imu;

void setup()
{
	Serial.begin(115200);// Start communication with a computer connected to Arduino via a USB cable
	Wire.begin(); // Start I2C
	
	imu.add(true); 
	imu.test();
}

void loop(){}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(String message) {
	Serial.print(message);
	while (true)
		;
}
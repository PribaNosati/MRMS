#include "VL53L1XsAnalog.h"
  
/**Add a sensor
@param pin - Analog pin, input for distance reading.
*/
void VL53L1XsAnalog::add(uint8_t pin) {
	if (nextFree >= MAX_VL53L1XS_ANALOG)
		error("Too many lidars.");
	pins[nextFree] = pin;
	nextFree++;
}

/**Distance
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Distance in mm.
*/
uint16_t VL53L1XsAnalog::distance(uint8_t sensorNumber){
	if (sensorNumber >= nextFree)
		error("Invalid sensor number");
	const float f = VL53L1XS_MAXIMUM_MM / (float)VL53L1XS_MAXIMUM_ANALOG_READ;
	return (uint16_t)(analogRead(pins[sensorNumber]) * f);
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void VL53L1XsAnalog::print(String message, bool eol) {
	if (eol) {
		Serial.println(message);
		if (serial != 0)
			serial->println(message);
	}
	else {
		Serial.print(message);
		if (serial != 0)
			serial->print(message);
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void VL53L1XsAnalog::test(BreakCondition breakWhen) {
	char buffer[30];
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (uint8_t i = 0; i < nextFree; i++) {
			sprintf(buffer, "%3i mm ", distance(i));
			print(buffer);
		}
		print("", true);
		delay(200);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
VL53L1XsAnalog::VL53L1XsAnalog(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

VL53L1XsAnalog::~VL53L1XsAnalog(){}
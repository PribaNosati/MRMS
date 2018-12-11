#include "IMUBoschBNO055Analog.h"
  
/**Add a sensor
@param pinHeadingSet - Analog pin, input for heading.
@param pinPitchSet - Analog pin, input for pitch.
@param pinRollSet - Analog pin, input for roll.
*/
void IMUBoschBNO055Analog::add(uint8_t pinHeadingSet, uint8_t pinPitchSet, uint8_t pinRollSet) {
	if (nextFree >= MAX_IMU_BOSCH_BNO055_SENSORS_ANALOG)
		error("Too many Bosch IMUs.");
	pinHeading[nextFree] = pinHeadingSet;
	pinPitch[nextFree] = pinPitchSet;
	pinRoll[nextFree] = pinRollSet;
	nextFree++;
}

/**Compass
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - North direction.
*/
float IMUBoschBNO055Analog::heading(uint8_t sensorNumber) {
	if (sensorNumber >= nextFree)
		error("Invalid sensor number");
	const float f = 360.0 / (float)MAXIMUM_ANALOG_READ;
	return analogRead(pinHeading[sensorNumber]) * f;
}

/**Pitch
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Pitch in degrees. Inclination forwards or backwards.
*/
float IMUBoschBNO055Analog::pitch(uint8_t sensorNumber) {
	if (sensorNumber >= nextFree)
		error("Invalid sensor number");
	if (pinPitch[sensorNumber] == 0xFF)
		return 0;
	else
		return analogRead(pinPitch[sensorNumber]);
}

/**Roll
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Roll in degrees. Inclination to the left or right.
*/
float IMUBoschBNO055Analog::roll(uint8_t sensorNumber) {
	if (sensorNumber >= nextFree)
		error("Invalid sensor number");
	if (pinRoll[sensorNumber] == 0xFF)
		return 0;
	else
		return analogRead(pinRoll[sensorNumber]);
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void IMUBoschBNO055Analog::print(String message, bool eol) {
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
void IMUBoschBNO055Analog::test(BreakCondition breakWhen) {
	char buffer[14];
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (uint8_t i = 0; i < nextFree; i++) {
			sprintf(buffer, "H:%3i P:%3i R:%3i", (int)round(heading(i)), (int)round(pitch(i)), (int)round(roll(i)));
			print(buffer);
		}
		print("", true);
		delay(200);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
IMUBoschBNO055Analog::IMUBoschBNO055Analog(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

IMUBoschBNO055Analog::~IMUBoschBNO055Analog(){}
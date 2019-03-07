#include "IRFinders.h"

/**Add a sensor
@param angle - Analog pin for angle. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
@param distance - analog pin for distance.
*/
void IRFinders::add(uint8_t anglePin, uint8_t distancePin)
{
	if (nextFree >= MAX_IR_FINDERS)
		error("Too many IR finders");
	anglePins[nextFree] = anglePin;
	distancePins[nextFree] = distancePin;
	nextFree++;
}

/** Does a light source exist (like an RCJ IR ball) or not?
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Exists or not.
*/
bool IRFinders::anyIRSource(uint8_t sensorNumber) {

	return analogRead(distancePins[sensorNumber]) > IR_FINDERS_DETECTION_THRESHOLD;
}

/** Angle of the light source (like an RCJ IR ball)
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Angle in degrees. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
*/
IRSource IRFinders::irSource(uint8_t sensorNumber) {
	IRSource source;

	source.angle = map(analogRead(anglePins[sensorNumber]), 0, IR_FINDERS_MAXIMUM_ANALOG_READ, -180, 180);
	source.distance = analogRead(distancePins[sensorNumber]);
	source.any = false;

	return source;
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void IRFinders::print(String message, bool eol) {
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
@param updateByTimerInterrupts - If so, no update() will be called in the function.
*/
void IRFinders::test(BreakCondition breakWhen, bool updateByTimerInterrupts) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			if (i != 0)
				print("  ");
			IRSource source = irSource(i);
			char buffer[] = "No source detected.        ";
			if (anyIRSource())
				sprintf(buffer, "Angle: %4dº, distance: %4d.", source.angle, source.distance);
			print(buffer);
			if (i == nextFree - 1)
				print("", true);
		}
		
		delay(300);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
IRFinders::IRFinders(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

IRFinders::~IRFinders(){}
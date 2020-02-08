#include "mrm-ir-finder2.h"

extern char errorMessage[];

/**Add a sensor
@param angle - Analog pin for angle. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
@param distance - analog pin for distance.
*/
void Mrm_ir_finder2::add(uint8_t anglePin, uint8_t distancePin)
{
	if (nextFree >= MAX_IR_FINDER2s) {
		strcpy(errorMessage, "Too many mrm-ir-finder2");
		return;
	}
	anglePins[nextFree] = anglePin;
	distancePins[nextFree] = distancePin;
	nextFree++;
}

/** Does a light source exist (like an RCJ IR ball) or not?
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Exists or not.
*/
bool Mrm_ir_finder2::anyIRSource(uint8_t sensorNumber) {

	return analogRead(distancePins[sensorNumber]) > IR_FINDERS_DETECTION_THRESHOLD;
}

/** Angle of the light source (like an RCJ IR ball)
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
@return - Angle in degrees. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
*/
IRSource Mrm_ir_finder2::irSource(uint8_t sensorNumber) {
	IRSource source;

	source.angle = map(analogRead(anglePins[sensorNumber]), 0, IR_FINDERS_MAXIMUM_ANALOG_READ, 0, -360);
	if (source.angle < -180)
		source.angle += 360;
	source.distance = analogRead(distancePins[sensorNumber]);
	source.any = false;

	return source;
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Mrm_ir_finder2::print(String message, bool eol) {
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
void Mrm_ir_finder2::test(BreakCondition breakWhen, bool updateByTimerInterrupts) {
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		if (nextFree == 0) {
			print("No mrm-ir-finder2\n\r");
			lastMs = 0xFFFFFFFFFFFFFFFF;
		}
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (pass++)
				print(" ");
			IRSource source = irSource(deviceNumber);
			char buffer[] = "No source detected.                                                      ";
			if (anyIRSource())
				sprintf(buffer, "Angle: %4iº, distance: %4i (an: %i/%i).", source.angle, source.distance, analogRead(anglePins[deviceNumber]), analogRead(distancePins[deviceNumber]));
			print(buffer);
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_ir_finder2::Mrm_ir_finder2(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_ir_finder2::~Mrm_ir_finder2(){}
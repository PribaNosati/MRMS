#include "mrm-ir-finder2.h"
#include <mrm-robot.h>

/**Constructor
@param robot - robot containing this board
*/
Mrm_ir_finder2::Mrm_ir_finder2(Robot* robot) {
	robotContainer = robot;
	nextFree = 0;
}

Mrm_ir_finder2::~Mrm_ir_finder2() {}

/**Add a sensor
@param angle - Analog pin for angle. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
@param distance - analog pin for distance.
*/
void Mrm_ir_finder2::add(uint8_t anglePin, uint8_t distancePin)
{
	if (nextFree >= MAX_IR_FINDER2s) {
		strcpy(robotContainer->errorMessage, "Too many mrm-ir-finder2");
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


/**Test
@param updateByTimerInterrupts - If so, no update() will be called in the function.
*/
void Mrm_ir_finder2::test(bool updateByTimerInterrupts) {
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		if (nextFree == 0) {
			robotContainer->print("No mrm-ir-finder2\n\r");
			lastMs = 0xFFFFFFFFFFFFFFFF;
		}
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (pass++)
				robotContainer->print(" ");
			IRSource source = irSource(deviceNumber);
			char buffer[] = "No source detected.                                                      ";
			if (anyIRSource())
				sprintf(buffer, "Angle: %4iº, distance: %4i (an: %i/%i).", source.angle, source.distance, analogRead(anglePins[deviceNumber]), analogRead(distancePins[deviceNumber]));
			robotContainer->print(buffer);
		}
		lastMs = millis();
		if (pass)
			robotContainer->print("\n\r");
	}
}
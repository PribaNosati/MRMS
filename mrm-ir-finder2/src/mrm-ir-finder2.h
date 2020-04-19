#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: Using MRMS IR finder. Separate sensors can be used or as a group, for detection of a RCJ ball.
@author MRMS team
@version 0.1 2019-09-27
Licence: You can use this code any way you like.
*/

#define MAX_IR_FINDER2s 1 //Maximum number of IR finders.
#define IR_FINDERS_DETECTION_THRESHOLD 50 // Increasing this number mitigates noise problems but decreases sensitivity
#define IR_FINDERS_MAXIMUM_ANALOG_READ 3300 // Change this number according to Your system in order to get exact angles. 
	//Numbers close to 1023 are expected for 10-bit ADCs and around 3500 for 12-bit ADCs (ESP32).

struct IRSource {
public:
	int16_t angle;
	uint16_t distance;
	bool any;
};

class Mrm_ir_finder2
{
	int nextFree;
	uint8_t anglePins[MAX_IR_FINDER2s];
	uint8_t distancePins[MAX_IR_FINDER2s];
	Robot* robotContainer;

public:
	/**Constructor
	@param robot - robot containing this board
	*/
	Mrm_ir_finder2(Robot* robot);

	~Mrm_ir_finder2();

	/**Add a sensor
	@param anglePin - Analog pin for angle. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
	@param distancePin - analog pin for distance.
	*/
	void add(uint8_t anglePin, uint8_t distancePin);

	/** Does a light source exist (like an RCJ IR ball) or not?
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	@return - Exists or not.
	*/
	bool anyIRSource(uint8_t sensorNumber = 0);

	/** Angle of the light source (like an RCJ IR ball)
	@return - Angle in degrees. Robot front is 0 degrees and positive angles are to the right. Total range is -180º to 180º.
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	*/
	IRSource irSource(uint8_t sensorNumber = 0);

	/**Test
	@param updateByTimerInterrupts - If so, no update() will be called in the function.
	*/
	void test(bool updateByTimerInterrupts = false);
};

#pragma once
#include "Arduino.h"

/**
Purpose: Using MRMS IR finder. Separate sensors can be used or as a group, for detection of a RCJ ball.
@author MRMS team
@version 0.0 2019-03-07
Licence: You can use this code any way you like.
*/

#define MAX_IR_FINDERS 1 //Maximum number of IR finders.
#define IR_FINDERS_DETECTION_THRESHOLD 1 // Increasing this number mitigates noise problems but decreases sensitivity
#define IR_FINDERS_MAXIMUM_ANALOG_READ 800 // Change this number according to Your system in order to get exact angles. Numbers close to 1023 are expected for 10-bit ADCs.
typedef bool(*BreakCondition)();

struct IRSource {
public:
	int16_t angle;
	uint16_t distance;
	bool any;
};

class IRFinders
{
	int nextFree;
	uint8_t anglePins[MAX_IR_FINDERS];
	uint8_t distancePins[MAX_IR_FINDERS];
	HardwareSerial * serial; //Additional serial port

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	IRFinders(HardwareSerial * hardwareSerial = 0);

	~IRFinders();

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
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	@param updateByTimerInterrupts - If so, no update() will be called in the function.
	*/
	void test(BreakCondition breakWhen = 0, bool updateByTimerInterrupts = false);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

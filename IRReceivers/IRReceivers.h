#pragma once
#include "Arduino.h"

/**
Purpose: Using MRMS IR detector. Separate sensors can be used or as a group, for detection of a RCJ ball.
@author MRMS team
@version 0.2 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_IR_RECEIVERS 20 //Maximum number of IR receivers. 
#define SAMPLE_LENGTH 100 //A bigger number increases precision but takes more time.
typedef bool(*BreakCondition)();

struct IRSource {
public:
	float angle;
	bool any;
};

class IRReceivers
{
	double angles[MAX_IR_RECEIVERS]; // Angle in degrees. Robot front is 0 degrees and positive angles are to the right.
	uint16_t cumulatives[MAX_IR_RECEIVERS];
	int nextFree;
	byte pins[MAX_IR_RECEIVERS]; //Digital pins the sensor use
	HardwareSerial * serial; //Additional serial port

	/** Index of a sensor left to the current.
	@param currentIndex - index of the current sensor.
	@return - index of a left sensor.
	*/
	uint8_t indexLeft(uint8_t currentIndex);

	/** Index of a sensor right to the current.
	@param currentIndex - index of the current sensor.
	@return - index of a right sensor.
	*/
	uint8_t indexRight(uint8_t currentIndex);
	
	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	IRReceivers(HardwareSerial * hardwareSerial = 0);

	~IRReceivers();

	/**Add a sensor
	@param pin - Digital pin the sensor uses
	@param angleDegrees - Angle in degrees. Robot front is 0 degrees and positive angles are to the right.
	*/
	void add(byte pin, double angleDegrees = 0);

	/** Does a light source exist (like an RCJ IR ball) or not?
	@return - Exists or not.
	*/
	bool anyIRSource(uint16_t threshold = 0);

	/** Angle of the light source (like an RCJ IR ball)
	@return - Angle in degrees. Robot front is 0 degrees and positive angles are to the right.
	*/
	IRSource irSource();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	@param updateByTimerInterrupts - If so, no update() will be called in the function.
	*/
	void test(BreakCondition breakWhen = 0, bool updateByTimerInterrupts = false);

	/** Periodically updates internal cumulatives.
	*/
	void update();
};

//Declaration of error function. Definition is in Your code.
void error(String message);

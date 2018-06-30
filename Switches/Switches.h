#pragma once
#include "Arduino.h"

/**
Purpose: reading switches.
@author MRMS team
@version 0.1 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_SWITCHES 4 // Maximum number of switches. 
typedef bool(*BreakCondition)();

class Switches
{
	uint8_t pins[MAX_SWITCHES]; // Digital pins switches use
	int nextFree;
	HardwareSerial * serial; //Additional serial port

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Switches(HardwareSerial * hardwareSerial = 0);

	~Switches();

	/** Add a switch
	@param pin - a digital pins the switch uses.
	*/
	void add(uint8_t pin);

	/** Pin
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first switch, 1 to second, etc.
	@return - Pin number
	*/
	uint8_t pin(uint8_t sensorNumber);

	/** Is on
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first switch, 1 to second, etc.
	@return - true if on.
	*/
	bool on(uint8_t sensorNumber);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

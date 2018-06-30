#pragma once
#include "Arduino.h"

/**
Purpoe: solenoid control.
@author MRMS team
@version 0.1 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_SOLENOIDS 2 // Maximum number of solenoids. 

typedef bool(*BreakCondition)();

class Solenoids
{
	uint8_t pins[MAX_SOLENOIDS]; // Digital pins solenoids use
	int nextFree;
	HardwareSerial * serial; //Additional serial port

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Solenoids(HardwareSerial * hardwareSerial = 0);

	~Solenoids();

	/** Add a solenoid
	@param pin - a digital pins the solenoid uses
	*/
	void add(uint8_t pin);

	/** Turn off
	@param solenoidNumber - Solenoid's index. Function add() assigns 0 to first solenoid, 1 to second, etc.
	*/
	void off(uint8_t solenoidNumber);

	/** Turn on
	@param solenoidNumber - Solenoid's index. Function add() assigns 0 to first solenoid, 1 to second, etc.
	*/
	void on(uint8_t solenoidNumber);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

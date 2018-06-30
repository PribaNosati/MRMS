#pragma once
#include "Arduino.h"

/**
Purpose: using encoders. Rotating motors with encoders increase their counters, so, for example, exact distance can be calculated.
@author MRMS team
@version 0.1 2018-05-19
Licence: You can use this code any way you like.
*/
#define MAX_ENCODERS 8 // Maximum number of encoders. 

typedef void(*ArgumenlessFunction)();
typedef bool(*BreakCondition)();

class Encoders
{
	uint16_t backupSteps[MAX_ENCODERS]; //For state restore.
	int nextFree;
	uint8_t pins[MAX_ENCODERS]; // Pins the encoders use.
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
	Encoders(HardwareSerial * hardwareSerial = 0);

	~Encoders();

	/**Add an encoder
	@param pin - Pin the encoder uses.
	*/
	void add(uint8_t pin);

	/**Backup positions
	*/
	void backup();

	/**Read a counter.
	@param encoderNumber - Encoder's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	uint32_t counter(int encoderNumber);

	/**Resets all the counters.
	@param to0 - reset to 0. Otherwise to backup. In that case, backup() function had to be called before.
	*/
	void reset(bool to0 = true);

	/**Set a counter.
	@param encoderNumber - Encoder's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param value - Value
	*/
	void set(int encoderNumber, uint32_t value);

	/**Test
	@param motorStartFunction - Motor starting function. It is necessary because encoders produce no results without the motors revolving.
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(ArgumenlessFunction motorStartFunction = 0, BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

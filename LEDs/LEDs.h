#pragma once
#include "Arduino.h"

/**
Purpose: LED switching on or off.
@author MRMS team
@version 0.1 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_LEDS 6 // Maximum number of LEDs. 

typedef bool(*BreakCondition)();

class LEDs
{
	uint8_t pins[MAX_LEDS]; // Digital pins the LEDs use
	bool isOn[MAX_LEDS];
	int nextFree;
	HardwareSerial * serial; //Additional serial port

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	LEDs(HardwareSerial * hardwareSerial = 0);

	~LEDs();

	/**Add a LED
	@param pin - Digital pin the LED uses
	*/
	void add(uint8_t pin);

	/**Turn off a LED
	@param ledNumber - LED index. Function add() assigns 0 to first LED, 1 to second, etc. 0xFF - turn off all LEDs.
	*/
	void off(uint8_t ledNumber = 0xFF);

	/**Turn on a LED
	@param ledNumber - LED index. Function add() assigns 0 to first LED, 1 to second, etc. 0xFF - turn on all LEDs.
	@param intensity - 0 - 255
	*/
	void on(uint8_t ledNumber = 0xFF, uint8_t intensity = 255);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

	/**Toggle u LED
	@param ledNumber - LED's index. Function add() assigns 0 to first LED, 1 to second, etc.
	*/
	void toggle(uint8_t ledNumber);
};

//Declaration of error function. Definition is in Your code.
void error(String message);



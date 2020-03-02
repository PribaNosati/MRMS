#pragma once
#include "Arduino.h"
#include <esp32-hal-ledc.h>
#include <mrm-board.h>

/**
Purpose: MRMS servo library
@author MRMS team
@version 0.0 2019-09-01
Licence: You can use this code any way you like.
For a deeper understanding check https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/ledc.html .
*/

#define MAX_SERVO_COUNT 10 // Maximum number of servo motors, the number cannot be bigger than 16 since there are 16 PWM channels in ESP32

typedef bool(*BreakCondition)();

class Mrm_servo
{
	char nameThis[MAX_SERVO_COUNT][10]; // Device's name
	int nextFree;
	uint8_t timerWidthBits;
	Robot* robotContainer;

public:
	/** Constructor
	@param robot - robot containing this board
	*/
	Mrm_servo(Robot* robot);

	~Mrm_servo();

	/**Add a servo motor
	@param gpioPin - pin number, allowed: 0 - 19, 21 - 23, 25 - 27, 32 - 36, 39
	@param deviceName - device's name
	@param timerWidth - timer width in bits, 1 - 16
	*/
	void add(uint8_t gpioPin = 16, char* deviceName = "", uint8_t timerWidth = 16);

	void sweep();

	/**Test
	*/
	void test();

	/** Move servo
	@param degrees - Servo's target angle, 0 - 180
	@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
	*/
	void write(uint16_t degrees = 90, uint8_t servoNumber = 0);
};



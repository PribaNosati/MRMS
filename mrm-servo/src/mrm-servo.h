#pragma once
#include "Arduino.h"
#include <esp32-hal-ledc.h>
#include <mrm-board.h>
#include "mrm-servo.h"

/**
Purpose: MRMS servo library
@author MRMS team
@version 0.0 2019-09-01
Licence: You can use this code any way you like.
For a deeper understanding check https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/ledc.html .
*/

// Maximum number of servo motors, the number cannot be bigger than 16 since there are 16 PWM channels in ESP32
#define MAX_SERVO_COUNT 16 // No more can fit in ESP32
#define MRM_SERVO_FREQUENCY_HZ 50 // Pulse occures FREQUENCY_HZ times each second. For 50 Hz period of one pulse is 20 ms.

typedef bool(*BreakCondition)();

class Mrm_servo
{
	std::vector<uint16_t>* _currentDegrees;
	std::vector<uint16_t>* _minDegrees;
	std::vector<float>* _minDegreesPulseMs;
	std::vector<uint16_t>* _maxDegrees;
	std::vector<float>* _maxDegreesPulseMs;
	std::vector<char[10]>* _name;// Device's name
	int nextFree;
	std::vector<uint8_t>* _timerWidth;
	Robot* robotContainer;
	float f;

public:
	/** Constructor
	@param robot - robot containing this board
	@param maxNumberOfServos - Maximum number of servo motors cannot be bigger than 16 since there are 16 PWM channels in ESP32
	*/
	Mrm_servo(Robot* robot = NULL, uint8_t maxNumberOfServos = 10);

	~Mrm_servo();

	/**Add a servo motor
	@param gpioPin - pin number, allowed: 0 - 19, 21 - 23, 25 - 27, 32 - 36, 39
	@param deviceName - device's name
	@param minDegrees - minimum servo angle
	@param maxDegrees - maximum servo angle
	@param minDegreesPulseMicroSec - pulse ms for minimum angle
	@param maxDegreesPulseMicroSec - pulse ms for maximum angle
	@param timerWidth - timer width in bits, 1 - 16. 12 yields angle resolution of about 1�
	*/
	void add(uint8_t gpioPin = 16, char* deviceName = (char *)"", uint16_t minDegrees = 0, uint16_t maxDegrees = 180, float minDegreesPulseMs = 1, float maxDegreesPulseMs = 2, uint8_t timerWidth = 12);

	void sweep();

	/**Test
	*/
	void test();

	/** Move servo
	@param degrees - Servo's target angle, 0 - 180�, or 0 - 360�, depending on model, counting clockwise
	@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
	@param ms - Duration of action in ms. 0 ms - immediately.
	*/
	void write(uint16_t degrees = 90, uint8_t servoNumber = 0, uint16_t ms = 0);

	/** Position servo according to user input.
	*/
	void writeInteractive();
};



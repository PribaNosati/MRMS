#pragma once
#include "Arduino.h"
#include <BluetoothSerial.h>
#include <esp32-hal-ledc.h>

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
	BluetoothSerial* serial; // Additional serial port

	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

public:
	/** Constructor
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_servo(BluetoothSerial* hardwareSerial = 0);

	~Mrm_servo();

	/**Add a servo motor
	@param gpioPin - pin number, allowed: 0 - 19, 21 - 23, 25 - 27, 32 - 36, 39
	@param deviceName - device's name
	@param timerWidth - timer width in bits, 1 - 16
	*/
	void add(uint8_t gpioPin = 16, char* deviceName = "", uint8_t timerWidth = 16);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

	/** Move servo
	@param degrees - Servo's target angle, 0 - 180
	@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
	*/
	void servoWrite(uint16_t degrees = 90, uint8_t servoNumber = 0);
};

//Declaration of error function. Definition is in Your code.
extern void error(char* message);



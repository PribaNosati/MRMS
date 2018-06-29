#pragma once
#include "Arduino.h"

/**
Purpose: using MRMS barrier for RCJ soccer.
@author MRMS team
@version 0.3 2018-05-13
Licence: You can use this code any way you like.
*/

#define MAX_BARRIERS 2 //Maximum number of barriers. 
#define TOP_EEPROM_ADDRESS_BARRIER 100//Start EEPROM address for calibration data. If You use EEPROM for other purposes, use different addresses.
//This program uses addresses TOP_EEPROM_ADDRESS_BARRIER till TOP_EEPROM_ADDRESS_BARRIER + 2 * MAX_BARRIERS.

typedef bool(*BreakCondition)();

class Barriers
{
	uint32_t lastTimeInterrupted = 0;
	uint16_t highestValues[MAX_BARRIERS];//Sensor's highest value
	uint16_t lowestValues[MAX_BARRIERS];//Sensor's lowest value
	int nextFree;
	byte pins[MAX_BARRIERS];//Analog pin the sensor uses.
	HardwareSerial * serial; //Additional serial port

	/**
	Writing into EEPROM
	*/
	void eepromWrite();

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication. Example:
		Barriers(&Serial2);
	*/
	Barriers(HardwareSerial * hardwareSerial = 0);

	~Barriers();

	/**Add a barrier
	@param pin - Analog pin. Barrier is connected here.
	@param lowestValue - The lowest value the sensor produces.
	@param highestValue - The highest.
	*/
	void add(byte pin, uint16_t lowestValue = 0, uint16_t highestValue = 0);

	/**Barrier calibration
	@param seconds - Calibration duration in seconds.
	*/
	void calibrate(uint16_t seconds = 5);

	/**
	EEPROM read
	*/
	void eepromRead();
	
	/**Barrier interrupted or not
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	@return - If interrupted, true.
	*/
	bool interrupt(uint8_t sensorNumber);

	/**Last interrupted time
	@return - Time (in ms) when the interrupt occured.*/
	uint32_t lastTimeInterrupt() { return lastTimeInterrupted; }

	/**Test
	@param numericValues - If true, display analog readings. If not, digital (X is interrupt).
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(bool numericValues = true, BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);


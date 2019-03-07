#pragma once
#include <Arduino.h>
/**
Purpose: Reading of STM VL53L1X lidars, analog output(s).
@author MRMS team
@version 0.0 2018-12-31
Licence: You can use this code any way you like.
*/
  
#define MAX_VL53L1XS_ANALOG 8 //Maximum number of sensors. 
#define VL53L1XS_MAXIMUM_ANALOG_READ 800 // Change this number according to Your system in order to get exact maximum distance
#define VL53L1XS_MAXIMUM_MM 4000.0 //Maximum distance in mm

typedef bool(*BreakCondition)();

class VL53L1XsAnalog
{
	int nextFree;
	uint8_t pins[MAX_VL53L1XS_ANALOG];
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
	VL53L1XsAnalog(HardwareSerial * hardwareSerial = 0);

	~VL53L1XsAnalog();

	/**Add a sensor
	@param pin - Analog pin, input for distance reading.
	*/
	void add(uint8_t pin);

	/**Distance
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	@return - Distance in mm.
	*/
	uint16_t distance(uint8_t sensorNumber = 0);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

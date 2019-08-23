#pragma once
#include "Arduino.h"
#include <i2c_t3.h>

/**
Svrha: reading of Devantech TPA81 thermal sensors.
@author MRMS team
@version 0.1 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_THERMAL_SENSORS_TPA81 6 // Maximum number of sensors.

typedef bool(*BreakCondition)();

class ThermalSensorsTPA81
{
	uint8_t addresses[MAX_THERMAL_SENSORS_TPA81]; // I2C addresses TPA81 use
	int nextFree;
	HardwareSerial * serial; //Additional serial port

	/** Read a sensor
	@param senzorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@param reg - Register. Check Devantech's documentation.
	@return - Result of reading.
	*/
	byte readSensor(byte sensorNumber, byte reg);
public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	ThermalSensorsTPA81(HardwareSerial * hardwareSerial = 0);

	~ThermalSensorsTPA81();

	/** Add a TPA81
	@param address - The sensor's I2C address
	*/
	void add(uint8_t address);

	/** Reads a temperature.
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@param rayNumber - Ray's number, 0 to 7.
	@return - Temperature in degrees Celsius.
	*/
	int temperature(uint8_t sensorNumber, uint8_t rayNumber);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);
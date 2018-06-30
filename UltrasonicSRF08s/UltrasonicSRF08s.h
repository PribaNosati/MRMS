#pragma once
#include <i2c_t3.h>

/**
Purpose: reading of Devantech SRF08 ultrasonic sensors.
@author MRMS team
@version 0.1 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_ULTRASONIC_SRF08 8 // Maximum number of sensors. 

typedef bool(*BreakCondition)();

class UltrasonicSRF08s
{
	uint8_t addresses[MAX_ULTRASONIC_SRF08]; // I2C addresses the sensors use
	bool enabled[MAX_ULTRASONIC_SRF08]; // Enabled
	uint16_t lastDistance[MAX_ULTRASONIC_SRF08]; // The last measured distance
	uint32_t lastPulseMs[MAX_ULTRASONIC_SRF08]; // The last time sensors triggered
	uint32_t lastReadMs[MAX_ULTRASONIC_SRF08]; // The last time returning wave's duration measured
	int nextFree;
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
	UltrasonicSRF08s(HardwareSerial * hardwareSerial = 0);

	~UltrasonicSRF08s();

	/** Add a SRF08
	@param address - An address the SRF08 uses.
	@param enabled - Enabled
	*/
	void add(uint8_t address, bool enabled = true);

	/** Distance in cm, uses polling. The program must wait for a return wave - very slow.
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@return - Distance.
	*/
	uint16_t distance(uint8_t sensorNumber);

	/** Distance in cm, asynchronous. The program will not wait for a return wave - very fast.
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@param cacheMaxMs - Maximum cache time, starting when triggered.
	@return - Distance.
	*/
	uint16_t distanceAsync(uint8_t sensorNumber, uint32_t msSincePulseAllowed = 500);

	/** By using echo values, calculate distance in cm
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@return - Distance
	*/
	uint16_t echo(uint8_t sensorNumber);

	/** Trigger a sound wave
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	*/
	void pulse(uint8_t sensorNumber);

	/** Set range
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@param maxGain - Maximum amplifier gain, 0x00 - 0x1F
	@param range - Defines time the sensor waits for return wave: (range + 1) * 43 mm
	*/
	void rangeAndGain(uint8_t sensorNumber, uint8_t maxGain, uint8_t range);

	/** Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	@param async - Asynchronous mode.
	*/
	void test(BreakCondition breakWhen = 0, bool async = false);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

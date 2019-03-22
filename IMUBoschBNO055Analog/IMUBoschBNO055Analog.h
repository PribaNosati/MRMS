#pragma once
#include <Arduino.h>
/**
Purpose: Reading of Bosch BNO055 IMU sensor, analog output(s).
@author MRMS team
@version 0.0 2018-12-11
Licence: You can use this code any way you like.
*/
  
#define MAX_IMU_BOSCH_BNO055_SENSORS_ANALOG 1 //Maximum number of IMUs. 
#define BOSCH_BNO055_MAXIMUM_ANALOG_READ 1000 // Change this number according to Your system in order to get maximum reading 360 degrees

typedef bool(*BreakCondition)();

class IMUBoschBNO055Analog
{
	int nextFree;
	uint8_t pinHeading[MAX_IMU_BOSCH_BNO055_SENSORS_ANALOG];
	uint8_t pinPitch[MAX_IMU_BOSCH_BNO055_SENSORS_ANALOG];
	uint8_t pinRoll[MAX_IMU_BOSCH_BNO055_SENSORS_ANALOG];
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
	IMUBoschBNO055Analog(HardwareSerial * hardwareSerial = 0);

	~IMUBoschBNO055Analog();

	/**Add a sensor
	@param pinHeadingSet - Analog pin, input for heading.
	@param pinPitchSet - Analog pin, input for pitch.
	@param pinRollSet - Analog pin, input for roll.
	*/
	void add(uint8_t pinHeadingSet, uint8_t pinPitchSet = 0xFF, uint8_t pintRollSet = 0xFF);

	/**Compass
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	@return - North direction.
	*/
	float heading(uint8_t sensorNumber = 0);

	/**Pitch
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	@return - Pitch in degrees. Inclination forwards or backwards.
	*/
	float pitch(uint8_t sensorNumber = 0);

	/**Roll
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns an increasing number to the sensor, starting with 0.
	@return - Roll in degrees. Inclination to the left or right.
	*/
	float roll(uint8_t sensorNumber = 0);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

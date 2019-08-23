#pragma once
#include <Wire.h>
#include <BluetoothSerial.h>
#include "bno055.h"// bno055.h and bno055.cpp files must be in Arduino libraries.

/**
Purpose: Reading of Bosch BNO055 IMU sensor.
@author MRMS team
@version 0.1 2018-08-18
Licence: You can use this code any way you like.
*/

#define MAX_MRM_IMU 1 //Maximum number of IMUs. 

typedef bool(*BreakCondition)();

class Mrm_imu
{
	bool defaultI2CAddresses[MAX_MRM_IMU]; //If true, it will use default I2C address (0x29) otherwise 0x28.
	struct bno055_t bno055; // Structure declaration.
	int nextFree;
	BluetoothSerial* serial; // Additional serial port

	void bno055Initialize(bool defaultI2CAddress = true); //IMU initialization of the sensor. It should be called once, after Wire.begin(). 
	s32 bno055_data_readout_template(void); //A function You do not use.
	
	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_imu(BluetoothSerial* hardwareSerial = 0);

	~Mrm_imu();

	/** Acceleration calibration
	@return - Calibration
	*/
	uint8_t accelerationCalibration();

	/**Add a BNO05
	@param defautI2CAddress - If true, 0x29. Otherwise 0x28.
	*/
	void add(bool defautI2CAddress = true);

	/** Gyro calibration
	@return - Calibration
	*/
	uint8_t gyroCalibration();

	/**Compass
	@return - North direction.
	*/
	float heading();

	/** Magnetic calibration
	@return - Calibration
	*/
	uint8_t magneticCalibration();

	/**Pitch
	@return - Pitch in degrees. Inclination forwards or backwards.
	*/
	float pitch();

	/**Roll
	@return - Roll in degrees. Inclination to the left or right.
	*/
	float roll();

	/** System calibration
	@return - Calibration
	*/
	uint8_t systemCalibration();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
extern void error(char* message);
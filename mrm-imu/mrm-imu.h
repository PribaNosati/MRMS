#pragma once
#include <Wire.h>
#include "bno055.h"// bno055.h and bno055.cpp files must be in Arduino libraries.
#ifdef ESP_PLATFORM
#include <mrm-board.h>
#else
#include <Arduino.h>
#endif

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
#ifdef ESP_PLATFORM
	Robot* robotContainer;
#endif

	void bno055Initialize(bool defaultI2CAddress = true); //IMU initialization of the sensor. It should be called once, after Wire.begin(). 

	s32 bno055_data_readout_template(void); //A function You do not use.

	void errorGeneric() { Serial.println("Error."); }

	void testHelper();

public:
	/**Constructor
	@param robot - robot containing this board
	*/
#ifdef ESP_PLATFORM
	Mrm_imu(Robot* robot = NULL);
#else
	Mrm_imu();
#endif

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
	*/
	void test();
};

//Declaration of error function. Definition is in Your code.
extern void error(char* message);
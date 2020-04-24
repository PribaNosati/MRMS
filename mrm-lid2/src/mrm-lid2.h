#pragma once
#include "Arduino.h"
#include <Wire.h>
#include "VL53L1X_api.h"

/**
Purpose: using VL53L1X LIDARs.
@author MRMS team
@version 0.0 2018-05-09
Licence: You can use this code any way you like.
*/

#define MAX_VL53L1XS 12 // Maximum number of sensors. 

typedef void(*ArgumenlessFunction)();
typedef bool(*BreakCondition)();

class VL53L1Xs
{
	uint16_t lastDistance[MAX_VL53L1XS]; // Last distance in mm
	uint32_t lastMeasurement[MAX_VL53L1XS]; // Last distance measurement in ms
	int nextFree = 0; // Sensor count + 1
	uint8_t pins[MAX_VL53L1XS]; // Pins the LIDARs use for XSHUT.
	HardwareSerial * serial; // Additional serial port
	VL53L1_Dev_t  *pDev[MAX_VL53L1XS];
	bool warnings; // Display warnings.

	/** Error-handling function
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param message - error message.
	*/
	void errorVL(uint8_t sensorNumber, String message);

	/** Warning
	@param message
	*/
	void warning(String message);

  public:
	/**Short range - 1.3 m
	Middle range - 3 m
	Long range - 4 m
	*/
	enum RangeProfile { ShortRange, MidRange, LongRange };

	/** Constructor
	@param hardwareSerial - optional additional serial port, for example for Bluetooth.
	@param displayWarnings - display ranging warnings.
	*/
	VL53L1Xs(HardwareSerial * hardwareSerial = 0, bool displayWarnings = false);

	/** Destructor
	*/
	~VL53L1Xs();

	/** Add a sensor. It assigns sensor 0 to the first sensor, 1 to the second, etc. This number ("sensorNumber") is used to 
		call other functions for this sensor.
	@param pin - Sensor's enable pin. @param pin - Sensor's enable pin. 0xFF - not used. Sensor will be enabled if this pin if left 
		unconnected due to internal pull-up.
	@param i2c_add - I2C address. 0x29 must not be used to any other I2C device, even if not used for any VL53L1X.
	*/
	void add(uint8_t pin = 0xFF, uint8_t i2c_addr = 0x29);

	/** Starts sensors. It must be called after all the add() calls.
	@param continuousMeasurement - Non-stop measuring.
	@param verbose - Detailed display.
	*/
	void begin(boolean verbose = false);

	/** Distance measurement
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - Distance in mm.
	*/
	uint16_t distance(uint8_t sensorNumber);

	/** Ranging profile
	@param value - profile
	@param sensorNumber -  Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
					0xFF - all sensors.
	*/
	void rangeProfileSet(RangeProfile value, uint8_t sensorNumber = 0xFF);

	/** Region of interest, can used to set FOV (field of view). The minimum ROI size is 4x4, maximum (and default) 16x16. Use 
		roiCenter() to find center.
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting
		with 0. 0xFF - all sensors.
	@param TopLeftX - top left X of the FOV, 0-15.
	@param TopLeftY - top left X of the FOV, 0-15.
	@param BottomRightX - bottom right X of the FOV, 0-15.
	@param BottomRightY - bottom right X of the FOV, 0-15.
	*/
	void roi(uint8_t sensorNumber = 0xFF,  uint8_t TopLeftX = 0, uint8_t TopLeftY = 15, uint8_t BottomRightX = 15, uint8_t BottomRightY = 0);

	/** ROI center. Due to assembly tolerances, the optical center of the device can vary.
	@param xCenter - returns center's x coordinate, 0-15.
	@param yCenter - returns center's y coordinate, 0-15.
	*/
	void roiCenter(uint8_t sensorNumber, uint8_t &xCenter, uint8_t &yCenter);
	
	/** Stress test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the stress() will be interrupted.
	*/
	void stress(BreakCondition breakWhen = 0);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

/* Declaration of error function. Definition is in Your code. Recommended actions: stop the motors, display error message
and go into an endless loop.
@param message - error message.
*/
void error(String message);
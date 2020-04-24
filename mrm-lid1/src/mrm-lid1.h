#pragma once
#include "Arduino.h"
#include <Wire.h>
#include "vl53l0x_api.h"

/**
Purpose: using VL53L0X LIDARs.
@author MRMS team
@version 0.2 2018-05-30
Licence: You can use this code any way you like but watch for STM license's conditions.
*/

#define MAX_VL53L0XS 12 // Maximum number of sensors. 
#define SAMPLE_COUNT_AVG 0 // Sample count for averages measurement - it takes a lot of memory if not 0!

typedef void(*ArgumenlessFunction)();
typedef bool(*BreakCondition)();

class VL53L0Xs
{
#if SAMPLE_COUNT_AVG > 0
	uint16_t averageSamples[MAX_VL53L0XS][SAMPLE_COUNT_AVG];
	uint8_t averageCounter[MAX_VL53L0XS];
#endif
	bool continuous; // Ranging continuously
	uint16_t lastDistance[MAX_VL53L0XS]; // Last distance in mm
	uint32_t lastMeasurement[MAX_VL53L0XS]; // Last distance measurement in ms
	int nextFree = 0; // Sensor count + 1
	uint8_t pins[MAX_VL53L0XS]; // Pins the LIDARs use for XSHUT.
	HardwareSerial * serial; //Additional serial port
	VL53L0X_Dev_t *pDev[MAX_VL53L0XS];

	/** Error-handling function
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param message - error message.
	*/
	void errorVL(uint8_t sensorNumber, String message);

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

  public:
	/**
	Default mode: 30ms budget 1.2m, accuracy as per Table 12 in data sheet.
	High accuracy: 200ms budget 1.2m, accuracy < +/- 3% precise measurement.
	Long range: 33ms budget 2m, long ranging, only for dark conditions (no IR).
	High speed: 20ms budget 1.2m, accuracy +/- 5% high speed where accuracy is not priority.
	*/
	enum RangeProfile { DefaultMode, HighAccuracy, LongRange, HighSpeed };

	/** Constructor
	@param hardwareSerial - optional additional serial port, for example for Bluetooth.
	*/
	VL53L0Xs(HardwareSerial * hardwareSerial = 0);

	/** Add a sensor. It assigns sensor 0 to the first sensor, 1 to the second, etc. This number ("sensorNumber") is used to
		call other functions for this sensor.
	@param pin - Sensor's enable pin. @param pin - Sensor's enable pin. 0xFF - not used. Sensor will be enabled if this pin if left
	unconnected due to internal pull-up.
	@param i2c_addr - I2C address. 0x29 must not be used to any other I2C device, even if not used for any VL53L1X.
	*/
	void add(uint8_t pin, uint8_t i2c_addr = 0x29);

	/** Starts sensors. It must be called after all the add() calls.
	@param continuousMeasurement - Non-stop measuring.
	@param verbose - Detailed display.
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void begin(bool continuousMeasurement = true, boolean verbose = false, uint8_t sensorNumber = 0xFF);

	/** Distance measurement
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param average - if true, average measurement. Sample count is defined by using SAMPLE_COUNT_AVG.
	@return - Distance in mm.
	*/
	uint16_t distance(uint8_t sensorNumber, bool average = false);

	/** Ranging profile
	@param value - profile
	@param sensorNumber -  Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
					0xFF - all sensors.
	*/
	void rangeProfileSet(RangeProfile value, uint8_t sensorNumber = 0xFF);
	
	/** Reset
	@param sensorNumber - sensor number. 0xFF - all the sensors.
	*/
	void reset(uint8_t sensorNumber = 0xFF);

	/** Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	@param average - if true, average measurement. Sample count is defined by using SAMPLE_COUNT_AVG.
	*/
	void test(BreakCondition breakWhen = 0, bool average = false);
};

/* Declaration of error function. Definition is in Your code. Recommended actions: stop the motors, display error message
	and go into an endless loop.
@param message - error message.
*/
void error(String message);
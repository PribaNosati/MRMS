#pragma once
#include "Arduino.h"

/**
Purpose: using MRMS infrared distance sensors
@author MRMS team
@version 0.3 2018-05-12
Licence: You can use this code any way you like.
*/

#define MAX_IR_DISTANCE_SENSORS 16 //Maximum number of sensors. You can use a smaller number, with a small memory waste.

/*Start EEPROM address for calibration data. If You use EEPROM for other purposes, use different addresses. For example, if You use ReflectanceSensors class,
and TOP_EEPROM_ADDRESS_REFLECTANCE = 0,  MAX_REFLECTANCE_SENSORS = 20, first 40 (or 60) bytes will be used there. So,  TOP_EEPROM_ADDRESS_IR_DISTANCE
will have to be 40 (or 60) or more.*/
#define TOP_EEPROM_ADDRESS_IR_DISTANCE 50

//User defined distances and analog readings. You can use calibration instead, in which case these definitions are not used.
#define DISTANCE_1_IN_CM 3
#define DISTANCE_2_IN_CM 4
#define DISTANCE_3_IN_CM 5
#define DISTANCE_4_IN_CM 7
#define DISTANCE_5_IN_CM 9
#define DISTANCE_6_IN_CM 12
#define DISTANCE_7_IN_CM 15
#define DISTANCE_8_IN_CM 20
#define DISTANCE_9_IN_CM 25
#define DISTANCE_10_IN_CM 30
#define DISTANCE_11_IN_CM 40
#define DISTANCE_12_IN_CM 60
#define IR_READING_FOR_1 690
#define IR_READING_FOR_2 671
#define IR_READING_FOR_3 658
#define IR_READING_FOR_4 644
#define IR_READING_FOR_5 600
#define IR_READING_FOR_6 579
#define IR_READING_FOR_7 532
#define IR_READING_FOR_8 460
#define IR_READING_FOR_9 393
#define IR_READING_FOR_10 275
#define IR_READING_FOR_11 87
#define IR_READING_FOR_12 27

typedef bool(*BreakCondition)();

class IRDistanceSensors {
	static const int pointCount = 12;
	bool calibrated;
	uint16_t cm[pointCount]; //For array 'reading', for the same index, stores number of cm.
	bool enabled[MAX_IR_DISTANCE_SENSORS]; // Enabled
	byte pins[MAX_IR_DISTANCE_SENSORS]; //Analog pin the sensor uses.
	int nextFree;
	uint16_t reading[pointCount]; //Analog readings
	HardwareSerial * serial; //Additional serial port

	 /**
	 Reading from EEPROMa
	 @param verbose - If true, displays the read values.
	 */
	void eepromRead(bool verbose = false);

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
	/** Constructor
	@param useCalibrationFromEEPROM - If true, read calibration data from EEPROM. In that case calibration (calibrate()) must be run at least once. The results are stored premanently.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	IRDistanceSensors(bool useCalibrationFromEEPROM = false, HardwareSerial * hardwareSerial = 0);

	/** Add a sensor
	@param pin - Analog pin the sensor uses.
	@param enabled - Enabled
	*/
	void add(byte pin, bool enabled = true);

	/** Calibration, distances in cm for given analog values. 2 modes: with external calibration function supplied or without. If
		no function supplied, the obstacle will have to be moved by hand to exact distances. If supplied, the obstacles can be 
		moved slowly between minimum (3 cm) and maximum (60 cm) distances. A typical external function returns 
		distances measured by an ultrasonic sensor or a LIDAR, at the same position, and pointing in the same direction, as this 
		reflectance sensor.
	@param sensorNumber - Sensor's ordinal number, the one that is used for calibration (an obstacle must be in front of that one).
	@param externalCalibrationFunction - External function returning exact distance.
	*/
	void calibrate(uint8_t sensorNumber = 0, float(*externalCalibrationFunction)() = 0);

	/** Display calibration data
	@param sensorNumber - Sensor's ordinal number, the one that is used for calibration.
	*/
	void calibrationDisplay(uint8_t sensorNumber = 0);

	/** Set calibration
	@param readings - instead of EEPROM values, use external values for calibration
	*/
	void calibrationSet(uint16_t readings[]);

	/** Number of sensors
	@return - count.
	*/
	int count();

	/** Distance
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - Distance in cm.
	*/
	float distance(byte sensorNumber);

	/**Test
	@param numericValues - If true, numeric (analog) values will be displayed. If not, distance in cm.
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(bool numericValues, BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

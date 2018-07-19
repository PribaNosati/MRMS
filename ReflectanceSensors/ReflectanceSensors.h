#pragma once
#include "Arduino.h"

/**
Purpose: usage of MRMS reflective sensors. Separate sensors can be used or as a group, for example for a line following.
@author MRMS team
@version 0.4 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_REFLECTANCE_SENSORS 20 // Maximum number of sensors. 
#define FACTOR_FOR_DARK 0.35// 0.35 means that 0 - 35% values above dark reading will be declared dark and 35% - 100% above bright.

/*Start EEPROM address for calibration data. If You use EEPROM for other purposes, use different addresses. For example, if You use IRDistanceSensors class,
and TOP_EEPROM_ADDRESS_REFLECTANCE = 0,  MAX_REFLECTANCE_SENSORS = 20, first 40 (or 60) bytes will be used there. So,  TOP_EEPROM_ADDRESS_IR_DISTANCE
will have to be 40 (or 60) or more.*/
#define TOP_EEPROM_ADDRESS_REFLECTANCE 1

typedef bool(*BreakCondition)();
enum CalibrationType{ALL, BRIGHT_AND_DARK, DARK, BRIGHT, EXTRA_BRIGHT};

class ReflectanceSensors {
	bool autoCalibration; // Autocalibration, previous data in highestBrightValues and lowestValues will not be used.
	bool forLineCalculations[MAX_REFLECTANCE_SENSORS]; // If true, the sensor will be used in a calculation that finds line's center (for a line follower).
	uint16_t brightValues[MAX_REFLECTANCE_SENSORS]; // Sensors' highest or average bright values.
	uint16_t extraBrightValues[MAX_REFLECTANCE_SENSORS]; // Sensors' average extra bright values.
	uint16_t darkValues[MAX_REFLECTANCE_SENSORS]; // Sensors' lowest or average dark values.
	uint32_t lastTimeAnyBrightMs = 0;
	double mms[MAX_REFLECTANCE_SENSORS]; // Distances in millimeters from a longitudinal axis of the robot, for a line follower. Negative values are for the left sensors.
	int nextFree;
	double percentForLine; // Values below this percentage will be neglected (not used in a line calculation).
	uint8_t pins[MAX_REFLECTANCE_SENSORS]; // Analog pins the sensors use.
	HardwareSerial * serial; //Additional serial port

	/**	EEPROM start address
	@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright (like for a mirror).
	@return - address
	*/
	uint16_t eepromStartAddress(CalibrationType calibrationType);

	/**	Writing into EEPROM
	@param calibrationType - DARK - only dark. BRIGHT - only bright. EXTRA_BRIGHT - only extra bright.
	*/
	void eepromWrite(CalibrationType calibrationType);

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

	/**	Single value, dark, bright, or extra bright
	@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright  (like for a mirror).
	@return - Value
	*/
	uint16_t valueGet(CalibrationType calibrationType, uint8_t index);

	/**	Set single value, dark, bright, or extra bright
	@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright (like for a mirror).
	@param value - Value to be set
	*/
	void valueSet(CalibrationType calibrationType, uint8_t index, uint16_t value);

public:
	/** Constructor
	@param autoCalibration - 0 - no autocalibration. Any other number - the robot will be decreasing minimums and increasing maximums during its run, according to the read values.
	@param percentageForLine - A number between 0 and 1. If a percentage (a ratio: [currentValue - minimumValue / maximumValue - minimumValue) is bigger
							than percentageForLine, the ratio will influence line's center calculation.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication.
	*/
	ReflectanceSensors(int autoCalibration = 0, double percentForLine = 0.25, HardwareSerial * hardwareSerial = 0);

	/** Add a sensor
	@param pin - Analog pin the sensor uses.
	@param mm - Distances in millimeters from th longitudinal axis of the robot, for a line follower. Negative values are for left sensors.
	@param forLineCalculation - It will be included in the calculation for line following.
	@param lowestValue - Sensor's lowest values.
	@param highestValue - Sensor's highest value.
	@param forLineCalculation - If true, the sensor will be used in a calculation that finds line's center (for a line follower).
	*/
	void add(byte pin, double mm = 0, bool forLineCalculation = true, uint16_t lowestValue = 0, uint16_t highestBrightValue = 0, uint16_t highestExtraBrightValue = 0);

	/** Any sensor reads a bright surface, like white.
	@return - Any or not.
	*/
	bool anyBright();

	/** Line calibration
	@param seconds - Calibration will last so long.
	@param calibrationType - BRIGHT_AND_DARK: moving over dark and bright surfaces. DARK - only average dark. BRIGHT - only average bright. 
	Similar for EXTRA_BRIGHT (like a mirror or a metal foil).
	*/
	void calibrate(uint16_t seconds = 10, CalibrationType calibrationType = BRIGHT_AND_DARK);

	/** Display calibration data
	@param sensorNumber - Sensor's ordinal number, the one that is used for calibration.
	*/
	void calibrationDisplay(uint8_t sensorNumber = 0);

	/** Sensor count
	@return - count.
	*/
	int count();

	/** Detection of a faulty sensor - not in use yet.
	*/
	void detectFaulty();

	/**	Reading from EEPROMa
	@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright.
	*/
	void eepromRead(CalibrationType calibrationType = ALL);

	/** Finding a (black) line
	@param lineFound - At least one sensor detected the line, output parameter.
	@param nonLineFound -  At least one sensor did not detect the line, output parameter.
	@return - Line's center distance, in millimeters, measuring form the robot's longitudinal axis, if found. If not, the last center.
	*/
	double findLine(bool & lineFound, bool & nonLineFound);

	/** Sensor reads a dark surface (like black).
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first LED, 1 to second, etc.
	@param verbose - verbose output.
	@return - It is dark or not.
	*/
	bool isDark(byte sensorNumber, bool verbose = false);

	/** Sensor reads a very bright surface (like a mirror or a shiny metal).
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first LED, 1 to second, etc.
	@return - It is extra bright (reflected from a mirror or a metal foil) or not.
	*/
	bool isExtraBright(byte sensorNumber);

	/** The last any sensor read a bright surface.
	@return - Time in ms.*/
	uint32_t lastTimeAnyBright() { return lastTimeAnyBrightMs; }

	/** Name
	@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright.
	@return - Name
	*/
	String name(CalibrationType calibrationType);

	/**
	@return - Raw reading
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first LED, 1 to second, etc.
	*/
	uint16_t raw(uint8_t sensorNumber);

	/**Test
	@param numericValues - If true, displays analog values. If not, digital (X for a dark line, a space otherwise).
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(bool numericValues = true, BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

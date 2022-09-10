#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-col-b interface to CANBus.
@author MRMS team
@version 0.0 2021-02-07
Licence: You can use this code any way you like.
*/

#define CAN_ID_COL_B0_IN 0x380
#define CAN_ID_COL_B0_OUT 0x381
#define CAN_ID_COL_B1_IN 0x382
#define CAN_ID_COL_B1_OUT 0x383
#define CAN_ID_COL_B2_IN 0x384
#define CAN_ID_COL_B2_OUT 0x385
#define CAN_ID_COL_B3_IN 0x386
#define CAN_ID_COL_B3_OUT 0x387
#define CAN_ID_COL_B4_IN 0x388
#define CAN_ID_COL_B4_OUT 0x389
#define CAN_ID_COL_B5_IN 0x38A
#define CAN_ID_COL_B5_OUT 0x38B
#define CAN_ID_COL_B6_IN 0x38C
#define CAN_ID_COL_B6_OUT 0x38D
#define CAN_ID_COL_B7_IN 0x38E
#define CAN_ID_COL_B7_OUT 0x38F

//CANBus commands
#define MRM_COL_B_SENDING_COLORS_1_TO_3 0x06
#define MRM_COL_B_SENDING_COLORS_4_TO_6 0x07
#define MRM_COL_B_SENDING_COLORS_7_TO_9 0x08
#define MRM_COL_B_SENDING_COLORS_10_TO_11 0x09
#define MRM_COL_B_ILLUMINATION_CURRENT 0x50
#define MRM_COL_B_SWITCH_TO_HSV 0x51
#define MRM_COL_B_SWITCH_TO_8_COLORS 0x52
#define MRM_COL_B_SENDING_HSV 0x53
#define MRM_COL_B_INTEGRATION_TIME 0x54
#define MRM_COL_B_GAIN 0x55
#define MRM_COL_B_PATTERN_RECORD 0x56
#define MRM_COL_B_PATTERN_SENDING 0x57
#define MRM_COL_B_PATTERN_REQUEST 0x58
#define MRM_COL_B_PATTERN_ERASE 0x59

#define MRM_COL_B_COLORS 10
#define MRM_COL_B_INACTIVITY_ALLOWED_MS 10000
#define MRM_COL_B_PATTERN_COUNT 16

class Mrm_col_b : public SensorBoard
{
	std::vector<bool>* _hsv; // If not - 10 colors
	std::vector<uint8_t>* _hue;
	std::vector<uint8_t>* _patternBy8Colors;
	std::vector<uint8_t>* _patternByHSV;
	std::vector<uint32_t>* _patternRecognizedAtMs;
	std::vector<uint16_t[MRM_COL_B_COLORS]>* readings; // Analog readings of all sensors
	std::vector<uint8_t>* _saturation;
	std::vector<uint8_t>* _value;

	/** If 10-colors mode not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool colorsStarted(uint8_t deviceNumber);

	/** If HSV not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool hsvStarted(uint8_t deviceNumber);
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_col_b(Robot* robot = NULL, uint8_t maxNumberOfBoards = 4);

	~Mrm_col_b();

	/** Add a mrm-col-b board
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");

	/** Blue
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorBlue(uint8_t deviceNumber);

	/** Blue greenish
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorBlueGeenish(uint8_t deviceNumber);

	/** Blue violetish
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorBlueVioletish(uint8_t deviceNumber);

	/** Green
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 
	@return - color intensity
	*/
	uint16_t colorGreen(uint8_t deviceNumber);

	/** Near IR
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorNearIR(uint8_t deviceNumber);

	/** Orange
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorOrange(uint8_t deviceNumber);

	/** Red
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorRed(uint8_t deviceNumber);

	/** Violet
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 
	@return - color intensity
	*/
	uint16_t colorViolet(uint8_t deviceNumber);

	/** White
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorWhite(uint8_t deviceNumber);

	/** Yellow
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 
	@return - color intensity
	*/
	uint16_t colorYellow(uint8_t deviceNumber);

	/** Set gain
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	@param gainValue:
		0	0.5x
		1	1x
		2	2x
		3	4x
		4	8x
		5	16x
		6	32x
		7	64x
		8	128x
		9	256x (default)
		10	512x
	*/
	void gain(uint8_t deviceNumber = 0, uint8_t gainValue = 0);

	/** Hue
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - Hue
	*/
	uint8_t hue(uint8_t deviceNumber);

	/** Set illumination intensity
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	@param current - 0 - 3
	*/
	void illumination(uint8_t deviceNumber = 0, uint8_t current = 0);

	/** Set integration time
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	@param time - sets the ATIME parameter for integration time from 0 to 255, integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS.
	@param step - sets STEP.
	*/
	void integrationTime(uint8_t deviceNumber, uint8_t time, uint16_t step);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t length);

	/** Erase all patterns
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - in all sensors
	*/
	void patternErase(uint8_t deviceNumber = 0xFF);

	/** Print HSV patterns
	*/
	void patternPrint();

	/** Choose a pattern closest to the current 8 colors
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@raturn - patternNumber
	*/
	uint8_t patternRecognizedBy8Colors(uint8_t deviceNumber);

	/** Choose a pattern closest to the current HSV values
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param includeValue - if true, HSV compared. If not, HS.
	@raturn - patternNumber
	*/
	uint8_t patternRecognizedByHSV(uint8_t deviceNumber);

	/** Record a HSV pattern
	@param patternNumber - 0 - PATTERN_COUNT-1
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void patternRecord(uint8_t patternNumber = 0, uint8_t deviceNumber = 0);

	/** Record patterns manually
	*/
	void patternsRecord();

	/** Analog readings
	@param color - one of 10 colors
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t color = 0, uint8_t sensorNumber = 0);

	/** Saturation
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - saturation
	*/
	uint8_t saturation(uint8_t deviceNumber);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Instruction to sensor to switch to converting R, G, and B on board and return hue, saturation and value
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
	*/
	void switchToHSV(uint8_t deviceNumber = 0xFF);

	/** Instruction to sensor to start returning 10 raw colors
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
	*/
	void switchTo8Colors(uint8_t deviceNumber = 0xFF);

	/**Test
	@param hsv - if not, then 10 colors
	*/
	void test(bool hsvSelect);

	/** Value
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - value
	*/
	uint8_t value(uint8_t deviceNumber);

};



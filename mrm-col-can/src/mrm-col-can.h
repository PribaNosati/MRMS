#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-us interface to CANBus.
@author MRMS team
@version 0.1 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_COL_CAN0_IN 0x310
#define CAN_ID_COL_CAN0_OUT 0x311
#define CAN_ID_COL_CAN1_IN 0x312
#define CAN_ID_COL_CAN1_OUT 0x313
#define CAN_ID_COL_CAN2_IN 0x314
#define CAN_ID_COL_CAN2_OUT 0x315
#define CAN_ID_COL_CAN3_IN 0x316
#define CAN_ID_COL_CAN3_OUT 0x317
#define CAN_ID_COL_CAN4_IN 0x318
#define CAN_ID_COL_CAN4_OUT 0x319
#define CAN_ID_COL_CAN5_IN 0x31A
#define CAN_ID_COL_CAN5_OUT 0x31B
#define CAN_ID_COL_CAN6_IN 0x31C
#define CAN_ID_COL_CAN6_OUT 0x31D
#define CAN_ID_COL_CAN7_IN 0x31E
#define CAN_ID_COL_CAN7_OUT 0x31F

//CANBus commands
#define CAN_COL_SENDING_COLORS_1_TO_3 0x06
#define CAN_COL_SENDING_COLORS_4_TO_6 0x07
#define CAN_COL_ILLUMINATION_CURRENT 0x50
#define CAN_COL_SWITCH_TO_HSV 0x51
#define CAN_COL_SWITCH_TO_6_COLORS 0x52
#define CAN_COL_SENDING_HSV 0x53
#define CAN_COL_INTEGRATION_TIME 0x54
#define CAN_COL_GAIN 0x55
#define CAN_COL_PATTERN_RECORD 0x56
#define CAN_COL_PATTERN_SENDING 0x57
#define CAN_COL_PATTERN_REQUEST 0x58
#define CAN_COL_PATTERN_ERASE 0x59

#define MRM_COL_CAN_COLORS 6
#define MRM_COL_CAN_INACTIVITY_ALLOWED_MS 10000
#define MRM_COL_CAN_PATTERN_COUNT 16

class Mrm_col_can : public SensorBoard
{
	std::vector<bool>* _hsv;
	std::vector<uint8_t>* _hue;
	std::vector<uint8_t>* _patternBy6Colors;
	std::vector<uint8_t>* _patternByHSV;
	std::vector<uint32_t>* _patternRecognizedAtMs;
	std::vector<uint16_t[MRM_COL_CAN_COLORS]>* readings; // Analog readings of all sensors
	std::vector<uint8_t>* _saturation;
	std::vector<uint8_t>* _value;

	/** If 6-colors mode not started, start it and wait for 1. message
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
	Mrm_col_can(Robot* robot = NULL, uint8_t maxNumberOfBoards = 4);

	~Mrm_col_can();

	/** Add a mrm-col-can board
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");

	/** Blue
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorBlue(uint8_t deviceNumber);

	/** Green
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorGreen(uint8_t deviceNumber);

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

	/** Yellow
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - color intensity
	*/
	uint16_t colorYellow(uint8_t deviceNumber);

	/** Set gain
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	@param gainValue:
		0, 1x (default)
		1, 3.7x
		2, 16x
		3, 64x
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
	@param value - integration time will be value x 2.8 ms but double that in case of mode 2 (usual). value is between 0 and 255. value 18 is approx 10 FPS
	*/
	void integrationTime(uint8_t deviceNumber = 0, uint8_t value = 18);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Erase all patterns
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - in all sensors
	*/
	void patternErase(uint8_t deviceNumber = 0xFF);

	/** Print HSV patterns
	*/
	void patternPrint();

	/** Choose a pattern closest to the current 6 colors
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@raturn - patternNumber
	*/
	uint8_t patternRecognizedBy6Colors(uint8_t deviceNumber);

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
	@param color - one of 6 colors
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

	/** Instruction to sensor to start returning 6 raw colors
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
	*/
	void switchTo6Colors(uint8_t deviceNumber = 0xFF);

	/**Test
	@param hsv - if not, then 6 colors
	*/
	void test(bool hsvSelect);

	/** Value
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - value
	*/
	uint8_t value(uint8_t deviceNumber);

};



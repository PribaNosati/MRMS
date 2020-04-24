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
#define CAN_COL_ILLUMINATION_CURRENT 0x50

#define MRM_COL_CAN_COLORS 6

//CANBus commands
#define COMMAND_SENDING_COLORS_1_TO_3 0x06
#define COMMAND_SENDING_COLORS_4_TO_6 0x07

class Mrm_col_can : public SensorBoard
{
	std::vector<uint16_t[MRM_COL_CAN_COLORS]>* readings; // Analog readings of all sensors
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_col_can(Robot* robot = NULL, uint8_t maxNumberOfBoards = 4);

	~Mrm_col_can();

	/** Add a mrm-col-can board
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Blue
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - color intensity
	*/
	uint16_t colorBlue(uint8_t deviceNumber) { return (*readings)[deviceNumber][0]; }

	/** Green
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - color intensity
	*/
	uint16_t colorGreen(uint8_t deviceNumber) { return (*readings)[deviceNumber][1]; }

	/** Orange
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - color intensity
	*/
	uint16_t colorOrange(uint8_t deviceNumber) { return (*readings)[deviceNumber][2]; }

	/** Red
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - color intensity
	*/
	uint16_t colorRed(uint8_t deviceNumber) { return (*readings)[deviceNumber][3]; }

	/** Violet
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - color intensity
	*/
	uint16_t colorViolet(uint8_t deviceNumber) { return (*readings)[deviceNumber][4]; }

	/** Yellow
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - color intensity
	*/
	uint16_t colorYellow(uint8_t deviceNumber) { return (*readings)[deviceNumber][5]; }

	/** Set illumination intensity
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@param current - 0 - 3
	*/
	void illumination(uint8_t deviceNumber = 0, uint8_t current = 0);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Analog readings
	@param color - one of 6 colors
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t color = 0, uint8_t sensorNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	*/
	void test();

};



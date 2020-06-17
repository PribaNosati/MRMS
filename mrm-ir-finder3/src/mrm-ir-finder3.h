#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-ir-finder3 interface to CANBus.
@author MRMS team
@version 0.0 2020-06-08
Licence: You can use this code any way you like.
*/

#define ID_IR_FINDER3_0_IN 0x0330
#define ID_IR_FINDER3_0_OUT 0x0331
#define ID_IR_FINDER3_1_IN 0x0332
#define ID_IR_FINDER3_1_OUT 0x0333
#define ID_IR_FINDER3_2_IN 0x0334
#define ID_IR_FINDER3_2_OUT 0x0335
#define ID_IR_FINDER3_3_IN 0x0336
#define ID_IR_FINDER3_3_OUT 0x0337
#define ID_IR_FINDER3_4_IN 0x0338
#define ID_IR_FINDER3_4_OUT 0x0339
#define ID_IR_FINDER3_5_IN 0x033A
#define ID_IR_FINDER3_5_OUT 0x033B
#define ID_IR_FINDER3_6_IN 0x033C
#define ID_IR_FINDER3_6_OUT 0x033D
#define ID_IR_FINDER3_7_IN 0x033E
#define ID_IR_FINDER3_7_OUT 0x033F

#define MRM_IR_FINDER3_SENSOR_COUNT 18 // Number of IR receivers in each device.

//CANBus commands
#define COMMAND_IR_FINDER3_SENDING_SENSORS_1_TO_7 0x04
#define COMMAND_IR_FINDER3_SENDING_SENSORS_8_TO_12 0x05
#define COMMAND_IR_FINDER3_SENDING_ANGLE_AND_DISTANCE 0x09

class Mrm_ir_finder3 : public SensorBoard
{
	std::vector<uint16_t[MRM_IR_FINDER3_SENSOR_COUNT]>* readings; // Cumulative readings of all sensors
	int16_t angle;
	uint16_t distance;
	bool near;
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_ir_finder3(Robot* robot = 0, uint8_t maxNumberOfBoards = 1);

	~Mrm_ir_finder3();

	/** Add a mrm-ir-finder3 sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);
	
	/** Cumulative readings
	@param receiverNumberInSensor - single IR receiver in mrm-ir-finder3
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - cumulative value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	*/
	void test();

	/**Test
	*/
	void testCalculated();
};




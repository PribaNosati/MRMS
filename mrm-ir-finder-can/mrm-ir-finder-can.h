#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-ir-finder-can interface to CANBus.
@author MRMS team
@version 0.0 2019-10-26
Licence: You can use this code any way you like.
*/

#define CAN_ID_IR_FINDER_CAN0_IN 0x0290
#define CAN_ID_IR_FINDER_CAN0_OUT 0x0291
#define CAN_ID_IR_FINDER_CAN1_IN 0x0292
#define CAN_ID_IR_FINDER_CAN1_OUT 0x0293
#define CAN_ID_IR_FINDER_CAN2_IN 0x0294
#define CAN_ID_IR_FINDER_CAN2_OUT 0x0295
#define CAN_ID_IR_FINDER_CAN3_IN 0x0296
#define CAN_ID_IR_FINDER_CAN3_OUT 0x0297
#define CAN_ID_IR_FINDER_CAN4_IN 0x0298
#define CAN_ID_IR_FINDER_CAN4_OUT 0x0299
#define CAN_ID_IR_FINDER_CAN5_IN 0x029A
#define CAN_ID_IR_FINDER_CAN5_OUT 0x029B
#define CAN_ID_IR_FINDER_CAN6_IN 0x029C
#define CAN_ID_IR_FINDER_CAN6_OUT 0x029D
#define CAN_ID_IR_FINDER_CAN7_IN 0x029E
#define CAN_ID_IR_FINDER_CAN7_OUT 0x029F

#define MRM_IR_FINDER_CAN_SENSOR_COUNT 12 // Number of IR receivers in each device.

//CANBus commands
#define COMMAND_IR_FINDER_CAN_SENDING_SENSORS_1_TO_3 0x04
#define COMMAND_IR_FINDER_CAN_SENDING_SENSORS_4_TO_6 0x05
#define COMMAND_IR_FINDER_CAN_SENDING_SENSORS_7_TO_9 0x06
#define COMMAND_IR_FINDER_CAN_SENDING_SENSORS_10_TO_12 0x08
#define COMMAND_IR_FINDER_CAN_SENDING_ANGLE_AND_DISTANCE 0x09

class Mrm_ir_finder_can : public SensorBoard
{
	std::vector<uint16_t[MRM_IR_FINDER_CAN_SENSOR_COUNT]>* readings; // Cumulative readings of all sensors
	int16_t angle;
	uint16_t distance;
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_ir_finder_can(Robot* robot, uint8_t maxNumberOfBoards = 1);

	~Mrm_ir_finder_can();

	/** Add a mrm-ir-finder-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);
	
	/** Cumulative readings
	@param receiverNumberInSensor - single IR receiver in mrm-ir-finder-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - cumulative value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void testCalculated(BreakCondition breakWhen = 0);
};




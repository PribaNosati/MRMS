#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-us interface to CANBus.
@author MRMS team
@version 0.1 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_US0_IN 0x300
#define CAN_ID_US0_OUT 0x301
#define CAN_ID_US1_IN 0x302
#define CAN_ID_US1_OUT 0x303
#define CAN_ID_US2_IN 0x304
#define CAN_ID_US2_OUT 0x305
#define CAN_ID_US3_IN 0x306
#define CAN_ID_US3_OUT 0x307
#define CAN_ID_US4_IN 0x308
#define CAN_ID_US4_OUT 0x309
#define CAN_ID_US5_IN 0x30A
#define CAN_ID_US5_OUT 0x30B
#define CAN_ID_US6_IN 0x30C
#define CAN_ID_US6_OUT 0x30D
#define CAN_ID_US7_IN 0x30E
#define CAN_ID_US7_OUT 0x30F

#define MRM_US_ECHOES_COUNT 9

//CANBus commands


typedef bool(*BreakCondition)();

class Mrm_us : public SensorBoard
{
	std::vector<uint16_t[MRM_US_ECHOES_COUNT]>* readings; // Analog readings of all sensors
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_us(Robot* robot, uint8_t maxNumberOfBoards = 4);

	~Mrm_us();

	/** Add a mrm-us board
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Analog readings
	@param echoNumber - echo id
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t echoNumber = 0, uint8_t sensorNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};



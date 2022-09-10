#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-us1 interface to CANBus.
@author MRMS team
@version 0.0 2020-12-16
Licence: You can use this code any way you like.
*/

#define CAN_ID_US1_0_IN 0x370
#define CAN_ID_US1_0_OUT 0x371
#define CAN_ID_US1_1_IN 0x372
#define CAN_ID_US1_1_OUT 0x373
#define CAN_ID_US1_2_IN 0x374
#define CAN_ID_US1_2_OUT 0x375
#define CAN_ID_US1_3_IN 0x376
#define CAN_ID_US1_3_OUT 0x377
#define CAN_ID_US1_4_IN 0x378
#define CAN_ID_US1_4_OUT 0x379
#define CAN_ID_US1_5_IN 0x37A
#define CAN_ID_US1_5_OUT 0x37B
#define CAN_ID_US1_6_IN 0x37C
#define CAN_ID_US1_6_OUT 0x37D
#define CAN_ID_US1_7_IN 0x37E
#define CAN_ID_US1_7_OUT 0x37F

#define MRM_US1_INACTIVITY_ALLOWED_MS 10000


class Mrm_us1 : public SensorBoard
{
	std::vector<uint16_t>* readings; // Analog readings of all sensors

	/** If sensor not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool started(uint8_t deviceNumber);
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_us1(Robot* robot = NULL, uint8_t maxNumberOfBoards = 4);

	~Mrm_us1();

	/** Add a mrm-us1 board
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char *)"");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t length);

	/** Analog readings
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t sensorNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	*/
	void test();

};



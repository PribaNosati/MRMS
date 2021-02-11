#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-us-b interface to CANBus.
@author MRMS team
@version 0.0 2020-12-11
Licence: You can use this code any way you like.
*/

#define CAN_ID_US_B0_IN 0x360
#define CAN_ID_US_B0_OUT 0x361
#define CAN_ID_US_B1_IN 0x362
#define CAN_ID_US_B1_OUT 0x363
#define CAN_ID_US_B2_IN 0x364
#define CAN_ID_US_B2_OUT 0x365
#define CAN_ID_US_B3_IN 0x366
#define CAN_ID_US_B3_OUT 0x367
#define CAN_ID_US_B4_IN 0x368
#define CAN_ID_US_B4_OUT 0x369
#define CAN_ID_US_B5_IN 0x36A
#define CAN_ID_US_B5_OUT 0x36B
#define CAN_ID_US_B6_IN 0x36C
#define CAN_ID_US_B6_OUT 0x36D
#define CAN_ID_US_B7_IN 0x36E
#define CAN_ID_US_B7_OUT 0x36F

#define MRM_US_B_INACTIVITY_ALLOWED_MS 10000


class Mrm_us_b : public SensorBoard
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
	Mrm_us_b(Robot* robot = NULL, uint8_t maxNumberOfBoards = 4);

	~Mrm_us_b();

	/** Add a mrm-us-b board
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char *)"");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

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



#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-therm-b-can interface to CANBus.
@author MRMS team
@version 0.3 2019-09-07
Licence: You can use this code any way you like.
*/

#define CAN_ID_THERM_B_CAN0_IN 0x0210 
#define CAN_ID_THERM_B_CAN0_OUT 0x0211
#define CAN_ID_THERM_B_CAN1_IN 0x0212 
#define CAN_ID_THERM_B_CAN1_OUT 0x0213
#define CAN_ID_THERM_B_CAN2_IN 0x0214
#define CAN_ID_THERM_B_CAN2_OUT 0x0215
#define CAN_ID_THERM_B_CAN3_IN 0x0216
#define CAN_ID_THERM_B_CAN3_OUT 0x0217
#define CAN_ID_THERM_B_CAN4_IN 0x0218
#define CAN_ID_THERM_B_CAN4_OUT 0x0219
#define CAN_ID_THERM_B_CAN5_IN 0x021A
#define CAN_ID_THERM_B_CAN5_OUT 0x021B
#define CAN_ID_THERM_B_CAN6_IN 0x021C
#define CAN_ID_THERM_B_CAN6_OUT 0x021D
#define CAN_ID_THERM_B_CAN7_IN 0x021E
#define CAN_ID_THERM_B_CAN7_OUT 0x021F


class Mrm_therm_b_can : public SensorBoard
{
	std::vector<int16_t>* readings; // Highest temperature
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_therm_b_can(Robot* robot, uint8_t maxNumberOfBoards = 4);

	~Mrm_therm_b_can();

	/** Add a mrm-therm-b-an board
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);
		
	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	int16_t reading(uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};




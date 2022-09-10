#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-therm-b-can interface to CANBus.
@author MRMS team
@version 0.3 2019-09-07
Licence: You can use this code any way you like.
*/

#define CAN_ID_FET0_IN 0x0340 
#define CAN_ID_FET0_OUT 0x0341
#define CAN_ID_FET1_IN 0x0342 
#define CAN_ID_FET1_OUT 0x0343
#define CAN_ID_FET2_IN 0x0344
#define CAN_ID_FET2_OUT 0x0345
#define CAN_ID_FET3_IN 0x0346
#define CAN_ID_FET3_OUT 0x0347
#define CAN_ID_FET4_IN 0x0348
#define CAN_ID_FET4_OUT 0x0349
#define CAN_ID_FET5_IN 0x034A
#define CAN_ID_FET5_OUT 0x034B
#define CAN_ID_FET6_IN 0x034C
#define CAN_ID_FET6_OUT 0x034D
#define CAN_ID_FET7_IN 0x034E
#define CAN_ID_FET7_OUT 0x034F

#define COMMAND_TURN_ON 0x50
#define COMMAND_TURN_OFF 0x51

class Mrm_fet_can : public MotorBoard
{
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_fet_can(Robot* robot = NULL, uint8_t maxNumberOfBoards = 4);

	~Mrm_fet_can();

	/** Add a mrm-therm-b-an board
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t length);
		
	/**Test
	*/
	void test();

	/** Turn output on
	@outputNumber - 0 or 1
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 
	*/
	void turnOn(uint8_t outputNumber, uint8_t deviceNumber = 0);

	/** Turn output off
	@outputNumber - 0 or 1
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void turnOff(uint8_t outputNumber, uint8_t deviceNumber = 0);


};




#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-bldc2x50 interface to CANBus.
@author MRMS team
@version 0.3 2019-09-07
Licence: You can use this code any way you like.
*/

#define CAN_ID_BLDC2X5_0_MOTOR0_IN 0x0110
#define CAN_ID_BLDC2X5_0_MOTOR0_OUT 0x0111
#define CAN_ID_BLDC2X5_0_MOTOR1_IN 0x0112
#define CAN_ID_BLDC2X5_0_MOTOR1_OUT 0x0113

#define CAN_ID_BLDC2X5_1_MOTOR0_IN 0x0114
#define CAN_ID_BLDC2X5_1_MOTOR0_OUT 0x0115
#define CAN_ID_BLDC2X5_1_MOTOR1_IN 0x0116
#define CAN_ID_BLDC2X5_1_MOTOR1_OUT 0x0117

#define CAN_ID_BLDC2X5_2_MOTOR0_IN 0x0118
#define CAN_ID_BLDC2X5_2_MOTOR0_OUT 0x0119
#define CAN_ID_BLDC2X5_2_MOTOR1_IN 0x011A
#define CAN_ID_BLDC2X5_2_MOTOR1_OUT 0x011B

#define CAN_ID_BLDC2X5_3_MOTOR0_IN 0x011C
#define CAN_ID_BLDC2X5_3_MOTOR0_OUT 0x011D
#define CAN_ID_BLDC2X5_3_MOTOR1_IN 0x011E
#define CAN_ID_BLDC2X5_3_MOTOR1_OUT 0x011F

class Mrm_bldc2x50 : public MotorBoard
{
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_bldc2x50(Robot* robot = NULL, uint8_t maxNumberOfBoards = 2);

	~Mrm_bldc2x50();

	/** Add a motor attached to a mrm-bldc2x50 motor controller
	@param isReversed - changes rotation direction.
	@param isLeft - is on the left side
	@param deviceName - device's name
	*/
	void add(bool isReversed = false, char * deviceName = "");
};



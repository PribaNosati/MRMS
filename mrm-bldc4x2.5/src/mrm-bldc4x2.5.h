#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-bldc4x2.5 interface to CANBus.
@author MRMS team
@version 0.0 2019-08-16
Licence: You can use this code any way you like.
*/

#define CAN_ID_BLDC4X2_5_0_MOTOR0_IN 0x0240
#define CAN_ID_BLDC4X2_5_0_MOTOR0_OUT 0x0241
#define CAN_ID_BLDC4X2_5_0_MOTOR1_IN 0x0242
#define CAN_ID_BLDC4X2_5_0_MOTOR1_OUT 0x0243
#define CAN_ID_BLDC4X2_5_0_MOTOR2_IN 0x0244
#define CAN_ID_BLDC4X2_5_0_MOTOR2_OUT 0x0245
#define CAN_ID_BLDC4X2_5_0_MOTOR3_IN 0x0246
#define CAN_ID_BLDC4X2_5_0_MOTOR3_OUT 0x0247

#define CAN_ID_BLDC4X2_5_1_MOTOR0_IN 0x0248
#define CAN_ID_BLDC4X2_5_1_MOTOR0_OUT 0x0249
#define CAN_ID_BLDC4X2_5_1_MOTOR1_IN 0x024A
#define CAN_ID_BLDC4X2_5_1_MOTOR1_OUT 0x024B
#define CAN_ID_BLDC4X2_5_1_MOTOR2_IN 0x024C
#define CAN_ID_BLDC4X2_5_1_MOTOR2_OUT 0x024D
#define CAN_ID_BLDC4X2_5_1_MOTOR3_IN 0x024E
#define CAN_ID_BLDC4X2_5_1_MOTOR3_OUT 0x024F

class Mrm_bldc4x2_5 : public MotorBoard
{
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_bldc4x2_5(Robot* robot = NULL, uint8_t maxNumberOfBoards = 1);

	~Mrm_bldc4x2_5();

	/** Add a motor attached to a mrm-bldc2x50 motor controller
	@param isReversed - changes rotation direction.
	@param isLeft - is on the left side
	@param deviceName - device's name
	*/
	void add(bool isReversed = false, char * deviceName = (char*)"");
};



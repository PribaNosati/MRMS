#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-mot4x10 interface to CANBus.
@author MRMS team
@version 0.0 2019-08-18
Licence: You can use this code any way you like.
*/

#define CAN_ID_MOT4X10_0_MOTOR0_IN 0x0250
#define CAN_ID_MOT4X10_0_MOTOR0_OUT 0x0251
#define CAN_ID_MOT4X10_0_MOTOR1_IN 0x0252
#define CAN_ID_MOT4X10_0_MOTOR1_OUT 0x0253
#define CAN_ID_MOT4X10_0_MOTOR2_IN 0x0254
#define CAN_ID_MOT4X10_0_MOTOR2_OUT 0x0255
#define CAN_ID_MOT4X10_0_MOTOR3_IN 0x0256
#define CAN_ID_MOT4X10_0_MOTOR3_OUT 0x0257

#define CAN_ID_MOT4X10_1_MOTOR0_IN 0x0258
#define CAN_ID_MOT4X10_1_MOTOR0_OUT 0x0259
#define CAN_ID_MOT4X10_1_MOTOR1_IN 0x025A
#define CAN_ID_MOT4X10_1_MOTOR1_OUT 0x025B
#define CAN_ID_MOT4X10_1_MOTOR2_IN 0x025C
#define CAN_ID_MOT4X10_1_MOTOR2_OUT 0x025D
#define CAN_ID_MOT4X10_1_MOTOR3_IN 0x025E
#define CAN_ID_MOT4X10_1_MOTOR3_OUT 0x025F

class Mrm_mot4x10 : public MotorBoard
{
public:
	
	/** Constructor
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_mot4x10(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0, uint8_t maxNumberOfBoards = 1);

	~Mrm_mot4x10();

	/** Add a motor attached to a mrm-mot4x3.6can motor controller
	@param isReversed - changes rotation direction.
	@param isLeft - is on the left side
	@param deviceName - device's name
	*/
	void add(bool isReversed = false, char * deviceName = "");
};




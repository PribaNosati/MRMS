#pragma once
#include "Arduino.h"
#include <mrm-devices.h>

/**
Purpose: mrm-mot4x3.6can interface to CANBus.
@author MRMS team
@version 0.1 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_MOT4X3_6_CAN0_MOTOR0_IN 0x0230
#define CAN_ID_MOT4X3_6_CAN0_MOTOR0_OUT 0x0231
#define CAN_ID_MOT4X3_6_CAN0_MOTOR1_IN 0x0232
#define CAN_ID_MOT4X3_6_CAN0_MOTOR1_OUT 0x0233
#define CAN_ID_MOT4X3_6_CAN0_MOTOR2_IN 0x0234
#define CAN_ID_MOT4X3_6_CAN0_MOTOR2_OUT 0x0235
#define CAN_ID_MOT4X3_6_CAN0_MOTOR3_IN 0x0236
#define CAN_ID_MOT4X3_6_CAN0_MOTOR3_OUT 0x0237

#define CAN_ID_MOT4X3_6_CAN1_MOTOR0_IN 0x0238
#define CAN_ID_MOT4X3_6_CAN1_MOTOR0_OUT 0x0239
#define CAN_ID_MOT4X3_6_CAN1_MOTOR1_IN 0x023A
#define CAN_ID_MOT4X3_6_CAN1_MOTOR1_OUT 0x023B
#define CAN_ID_MOT4X3_6_CAN1_MOTOR2_IN 0x023C
#define CAN_ID_MOT4X3_6_CAN1_MOTOR2_OUT 0x023D
#define CAN_ID_MOT4X3_6_CAN1_MOTOR3_IN 0x023E
#define CAN_ID_MOT4X3_6_CAN1_MOTOR3_OUT 0x023F


class Mrm_mot4x3_6can : public MotorGroup
{
public:
	
	/** Constructor
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_mot4x3_6can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_mot4x3_6can();

	/** Add a motor attached to a mrm-mot4x3.6can motor controller
	@param isReversed - changes rotation direction.
	@param isLeft - is on the left side
	@param deviceName - device's name
	*/
	void add(bool isReversed = false, bool isLeft = true, char * deviceName = "");
};




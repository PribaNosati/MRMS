#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-mot2x50 interface to CANBus.
@author MRMS team
@version 0.0 2019-09-18
Licence: You can use this code any way you like.
*/

#define CAN_ID_MOT2X50_0_MOTOR0_IN 0x0260
#define CAN_ID_MOT2X50_0_MOTOR0_OUT 0x0261
#define CAN_ID_MOT2X50_0_MOTOR1_IN 0x0262
#define CAN_ID_MOT2X50_0_MOTOR1_OUT 0x0263
#define CAN_ID_MOT2X50_0_MOTOR2_IN 0x0264
#define CAN_ID_MOT2X50_0_MOTOR2_OUT 0x0265
#define CAN_ID_MOT2X50_0_MOTOR3_IN 0x0266
#define CAN_ID_MOT2X50_0_MOTOR3_OUT 0x0267

#define CAN_ID_MOT2X50_1_MOTOR0_IN 0x0268
#define CAN_ID_MOT2X50_1_MOTOR0_OUT 0x0269
#define CAN_ID_MOT2X50_1_MOTOR1_IN 0x026A
#define CAN_ID_MOT2X50_1_MOTOR1_OUT 0x026B
#define CAN_ID_MOT2X50_1_MOTOR2_IN 0x026C
#define CAN_ID_MOT2X50_1_MOTOR2_OUT 0x026D
#define CAN_ID_MOT2X50_1_MOTOR3_IN 0x026E
#define CAN_ID_MOT2X50_1_MOTOR3_OUT 0x026F


class Mrm_mot2x50 : public MotorBoard
{
public:
	
	/** Constructor
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_mot2x50(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_mot2x50();

	/** Add a motor attached to a mrm-mot4x3.6can motor controller
	@param isReversed - changes rotation direction.
	@param isLeft - is on the left side
	@param deviceName - device's name
	*/
	void add(bool isReversed = false, char * deviceName = "");
};




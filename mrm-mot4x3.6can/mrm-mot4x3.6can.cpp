#include "mrm-mot4x3.6can.h"

extern CAN_device_t CAN_cfg;
extern char errorMessage[];

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_mot4x3_6can::Mrm_mot4x3_6can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial, uint8_t maxNumberOfBoards) : 
	MotorBoard(esp32CANBusSingleton, 4, "Mot4x3.6", maxNumberOfBoards){
	serial = hardwareSerial;
}

Mrm_mot4x3_6can::~Mrm_mot4x3_6can()
{
}

/** Add a motor attached to a mrm-mot4x3.6can motor controller
@param isReversed - changes rotation direction.
@param isLeft - is on the left side
@param deviceName - device's name
*/
void Mrm_mot4x3_6can::add(bool isReversed, char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_MOT4X3_6_CAN0_MOTOR0_IN;
		canOut = CAN_ID_MOT4X3_6_CAN0_MOTOR0_OUT;
		break;
	case 1:
		canIn = CAN_ID_MOT4X3_6_CAN0_MOTOR1_IN;
		canOut = CAN_ID_MOT4X3_6_CAN0_MOTOR1_OUT;
		break;
	case 2:
		canIn = CAN_ID_MOT4X3_6_CAN0_MOTOR2_IN;
		canOut = CAN_ID_MOT4X3_6_CAN0_MOTOR2_OUT;
		break;
	case 3:
		canIn = CAN_ID_MOT4X3_6_CAN0_MOTOR3_IN;
		canOut = CAN_ID_MOT4X3_6_CAN0_MOTOR3_OUT;
		break;
	case 4:
		canIn = CAN_ID_MOT4X3_6_CAN1_MOTOR0_IN;
		canOut = CAN_ID_MOT4X3_6_CAN1_MOTOR0_OUT;
		break;
	case 5:
		canIn = CAN_ID_MOT4X3_6_CAN1_MOTOR1_IN;
		canOut = CAN_ID_MOT4X3_6_CAN1_MOTOR1_OUT;
		break;
	case 6:
		canIn = CAN_ID_MOT4X3_6_CAN1_MOTOR2_IN;
		canOut = CAN_ID_MOT4X3_6_CAN1_MOTOR2_OUT;
		break;
	case 7:
		canIn = CAN_ID_MOT4X3_6_CAN1_MOTOR3_IN;
		canOut = CAN_ID_MOT4X3_6_CAN1_MOTOR3_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-mot4x3.6");
		return;
	}
	MotorBoard::add(deviceName, canIn, canOut);
	(*reversed)[nextFree-1] = isReversed;
}
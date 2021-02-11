#include "mrm-bldc2x50.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_bldc2x50::Mrm_bldc2x50(Robot* robot, uint8_t maxNumberOfBoards) : MotorBoard(robot, 2, "Bldc2x50", maxNumberOfBoards, ID_MRM_BLDC2X50) {
}

Mrm_bldc2x50::~Mrm_bldc2x50()
{
}

/** Add a motor attached to a mrm-bldc2x50 motor controller
@param isReversed - changes rotation direction.
@param deviceName - device's name
*/
void Mrm_bldc2x50::add(bool isReversed, char * deviceName)
{
	uint16_t canIn = 0, canOut = 0;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_BLDC2X5_0_MOTOR0_IN;
		canOut = CAN_ID_BLDC2X5_0_MOTOR0_OUT;
		break;
	case 1:
		canIn = CAN_ID_BLDC2X5_0_MOTOR1_IN;
		canOut = CAN_ID_BLDC2X5_0_MOTOR1_OUT;
		break;
	case 2:
		canIn = CAN_ID_BLDC2X5_1_MOTOR0_IN;
		canOut = CAN_ID_BLDC2X5_1_MOTOR0_OUT;
		break;
	case 3:
		canIn = CAN_ID_BLDC2X5_1_MOTOR1_IN;
		canOut = CAN_ID_BLDC2X5_1_MOTOR1_OUT;
		break;
	case 4:
		canIn = CAN_ID_BLDC2X5_2_MOTOR0_IN;
		canOut = CAN_ID_BLDC2X5_2_MOTOR0_OUT;
		break;
	case 5:
		canIn = CAN_ID_BLDC2X5_2_MOTOR1_IN;
		canOut = CAN_ID_BLDC2X5_2_MOTOR1_OUT;
		break;
	case 6:
		canIn = CAN_ID_BLDC2X5_3_MOTOR0_IN;
		canOut = CAN_ID_BLDC2X5_3_MOTOR0_OUT;
		break;
	case 7:
		canIn = CAN_ID_BLDC2X5_3_MOTOR1_IN;
		canOut = CAN_ID_BLDC2X5_3_MOTOR1_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-bldc2x50");
	}
	MotorBoard::add(deviceName, canIn, canOut);

	(*reversed)[nextFree-1] = isReversed;
}

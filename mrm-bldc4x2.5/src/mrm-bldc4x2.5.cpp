#include "mrm-bldc4x2.5.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_bldc4x2_5::Mrm_bldc4x2_5(Robot* robot, uint8_t maxDevices) : MotorBoard(robot, 4, "Bld4x2.5", maxDevices, ID_MRM_BLDC4x2_5){
}

Mrm_bldc4x2_5::~Mrm_bldc4x2_5()
{
}

/** Add a motor attached to a mrm-bldc2x50 motor controller
@param isReversed - changes rotation direction.
@param isLeft - is on the left side
@param deviceName - device's name
*/
void Mrm_bldc4x2_5::add(bool isReversed, char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_BLDC4X2_5_0_MOTOR0_IN;
		canOut = CAN_ID_BLDC4X2_5_0_MOTOR0_OUT;
		break;
	case 1:
		canIn = CAN_ID_BLDC4X2_5_0_MOTOR1_IN;
		canOut = CAN_ID_BLDC4X2_5_0_MOTOR1_OUT;
		break;
	case 2:
		canIn = CAN_ID_BLDC4X2_5_0_MOTOR2_IN;
		canOut = CAN_ID_BLDC4X2_5_0_MOTOR2_OUT;
		break;
	case 3:
		canIn = CAN_ID_BLDC4X2_5_0_MOTOR3_IN;
		canOut = CAN_ID_BLDC4X2_5_0_MOTOR3_OUT;
		break;
	case 4:
		canIn = CAN_ID_BLDC4X2_5_1_MOTOR0_IN;
		canOut = CAN_ID_BLDC4X2_5_1_MOTOR0_OUT;
		break;
	case 5:
		canIn = CAN_ID_BLDC4X2_5_1_MOTOR1_IN;
		canOut = CAN_ID_BLDC4X2_5_1_MOTOR1_OUT;
		break;
	case 6:
		canIn = CAN_ID_BLDC4X2_5_1_MOTOR2_IN;
		canOut = CAN_ID_BLDC4X2_5_1_MOTOR2_OUT;
		break;
	case 7:
		canIn = CAN_ID_BLDC4X2_5_1_MOTOR3_IN;
		canOut = CAN_ID_BLDC4X2_5_1_MOTOR3_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mr-bldc4x2.5");
	}
	MotorBoard::add(deviceName, canIn, canOut);

	(*reversed)[nextFree-1] = isReversed;
}

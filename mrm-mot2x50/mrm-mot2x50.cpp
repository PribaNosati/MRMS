#include "mrm-mot2x50.h"

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_mot2x50::Mrm_mot2x50(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial, uint8_t maxDevices) : 
	MotorBoard(esp32CANBusSingleton, 4, "Mot4x3.6", maxDevices){
	serial = hardwareSerial;
}

Mrm_mot2x50::~Mrm_mot2x50()
{
}

/** Add a motor attached to a mrm-mot2x50 motor controller
@param isReversed - changes rotation direction.
@param isLeft - is on the left side
@param deviceName - device's name
*/
void Mrm_mot2x50::add(bool isReversed, char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_MOT2X50_0_MOTOR0_IN;
		canOut = CAN_ID_MOT2X50_0_MOTOR0_OUT;
		break;
	case 1:
		canIn = CAN_ID_MOT2X50_0_MOTOR1_IN;
		canOut = CAN_ID_MOT2X50_0_MOTOR1_OUT;
		break;
	case 2:
		canIn = CAN_ID_MOT2X50_0_MOTOR2_IN;
		canOut = CAN_ID_MOT2X50_0_MOTOR2_OUT;
		break;
	case 3:
		canIn = CAN_ID_MOT2X50_0_MOTOR3_IN;
		canOut = CAN_ID_MOT2X50_0_MOTOR3_OUT;
	case 4:
		canIn = CAN_ID_MOT2X50_1_MOTOR0_IN;
		canOut = CAN_ID_MOT2X50_1_MOTOR0_OUT;
		break;
	case 5:
		canIn = CAN_ID_MOT2X50_1_MOTOR1_IN;
		canOut = CAN_ID_MOT2X50_1_MOTOR1_OUT;
		break;
	case 6:
		canIn = CAN_ID_MOT2X50_1_MOTOR2_IN;
		canOut = CAN_ID_MOT2X50_1_MOTOR2_OUT;
		break;
	case 7:
		canIn = CAN_ID_MOT2X50_1_MOTOR3_IN;
		canOut = CAN_ID_MOT2X50_1_MOTOR3_OUT;
		break;
	default:
		error("Too many mrm-mot2x50s\n\r");
	}
	MotorBoard::add(deviceName, canIn, canOut);

	(*reversed)[nextFree-1] = isReversed;
}
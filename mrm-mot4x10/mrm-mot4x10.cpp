#include "mrm-mot4x10.h"

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_mot4x10::Mrm_mot4x10(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) : MotorGroup(esp32CANBusSingleton, 4, "Mot4x10"){
	serial = hardwareSerial;
}

Mrm_mot4x10::~Mrm_mot4x10()
{
}

/** Add a motor attached to a mrm-mot4x3.6can motor controller
@param isReversed - changes rotation direction.
@param isLeft - is on the left side
@param deviceName - device's name
*/
void Mrm_mot4x10::add(bool isReversed, bool isLeft, char * deviceName)
{
	MotorGroup::add(deviceName, CAN_ID_MOT4X10_0_MOTOR0_IN, CAN_ID_MOT4X10_0_MOTOR0_OUT, CAN_ID_MOT4X10_0_MOTOR1_IN, CAN_ID_MOT4X10_0_MOTOR1_OUT,
		CAN_ID_MOT4X10_0_MOTOR2_IN, CAN_ID_MOT4X10_0_MOTOR2_OUT, CAN_ID_MOT4X10_0_MOTOR3_IN, CAN_ID_MOT4X10_0_MOTOR3_OUT, CAN_ID_MOT4X10_1_MOTOR0_IN,
		CAN_ID_MOT4X10_1_MOTOR0_OUT, CAN_ID_MOT4X10_1_MOTOR1_IN, CAN_ID_MOT4X10_1_MOTOR1_OUT, CAN_ID_MOT4X10_1_MOTOR2_IN, CAN_ID_MOT4X10_1_MOTOR2_OUT,
		CAN_ID_MOT4X10_1_MOTOR3_IN, CAN_ID_MOT4X10_1_MOTOR3_OUT);

	reversed[nextFree-1] = isReversed;
	left[nextFree-1] = isLeft;
}
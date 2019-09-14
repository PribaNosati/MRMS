#include "mrm-bldc2x50.h"

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_bldc2x50::Mrm_bldc2x50(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) : MotorGroup(esp32CANBusSingleton, 2, "Mot2x50") {
	serial = hardwareSerial;
}

Mrm_bldc2x50::~Mrm_bldc2x50()
{
}

/** Add a motor attached to a mrm-bldc2x50 motor controller
@param isReversed - changes rotation direction.
@param isLeft - is on the left side
@param deviceName - device's name
*/
void Mrm_bldc2x50::add(bool isReversed, bool isLeft, char * deviceName)
{
	MotorGroup::add(deviceName, CAN_ID_BLDC2X5_0_MOTOR0_IN, CAN_ID_BLDC2X5_0_MOTOR0_OUT, CAN_ID_BLDC2X5_0_MOTOR1_IN, CAN_ID_BLDC2X5_0_MOTOR1_OUT, 
		CAN_ID_BLDC2X5_1_MOTOR0_IN, CAN_ID_BLDC2X5_1_MOTOR0_OUT, CAN_ID_BLDC2X5_1_MOTOR1_IN, CAN_ID_BLDC2X5_1_MOTOR1_OUT, CAN_ID_BLDC2X5_2_MOTOR0_IN, 
		CAN_ID_BLDC2X5_2_MOTOR0_OUT, CAN_ID_BLDC2X5_2_MOTOR1_IN, CAN_ID_BLDC2X5_2_MOTOR1_OUT, CAN_ID_BLDC2X5_3_MOTOR0_IN, CAN_ID_BLDC2X5_3_MOTOR0_OUT, 
		CAN_ID_BLDC2X5_3_MOTOR1_IN, CAN_ID_BLDC2X5_3_MOTOR1_OUT);

	reversed[nextFree-1] = isReversed;
	left[nextFree-1] = isLeft;
}

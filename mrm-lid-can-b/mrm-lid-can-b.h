#pragma once
#include "Arduino.h"
#include "mrm-devices.h"

/**
Purpose: mrm-lid-can-b interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_LID_CAN_B0_IN 0x0180
#define CAN_ID_LID_CAN_B0_OUT 0x0181
#define CAN_ID_LID_CAN_B1_IN 0x0182
#define CAN_ID_LID_CAN_B1_OUT 0x0183
#define CAN_ID_LID_CAN_B2_IN 0x0184
#define CAN_ID_LID_CAN_B2_OUT 0x0185
#define CAN_ID_LID_CAN_B3_IN 0x0186
#define CAN_ID_LID_CAN_B3_OUT 0x0187
#define CAN_ID_LID_CAN_B4_IN 0x0188
#define CAN_ID_LID_CAN_B4_OUT 0x0189
#define CAN_ID_LID_CAN_B5_IN 0x018A
#define CAN_ID_LID_CAN_B5_OUT 0x018B
#define CAN_ID_LID_CAN_B6_IN 0x018C
#define CAN_ID_LID_CAN_B6_OUT 0x018D
#define CAN_ID_LID_CAN_B7_IN 0x018E
#define CAN_ID_LID_CAN_B7_OUT 0x018F

//CANBus commands
#define COMMAND_LID_CAN_B_CALIBRATE 0x05


class Mrm_lid_can_b : public SensorGroup
{
	uint16_t readings[MAX_SENSORS_BASE]; // Analog readings of all sensors
	
public:
	
	/** Constructor
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_lid_can_b(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_lid_can_b();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");
	
	/** Calibration, only once after production
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void calibration(uint8_t deviceNumber = 0);
	
	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);
	
	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};



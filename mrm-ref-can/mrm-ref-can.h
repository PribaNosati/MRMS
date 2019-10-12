#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-ref-can interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_REF_CAN0_IN 0x0160
#define CAN_ID_REF_CAN0_OUT 0x0161
#define CAN_ID_REF_CAN1_IN 0x0162
#define CAN_ID_REF_CAN1_OUT 0x0163
#define CAN_ID_REF_CAN2_IN 0x0164
#define CAN_ID_REF_CAN2_OUT 0x0165
#define CAN_ID_REF_CAN3_IN 0x0166
#define CAN_ID_REF_CAN3_OUT 0x0167
#define CAN_ID_REF_CAN4_IN 0x0168
#define CAN_ID_REF_CAN4_OUT 0x0169
#define CAN_ID_REF_CAN5_IN 0x016A
#define CAN_ID_REF_CAN5_OUT 0x016B
#define CAN_ID_REF_CAN6_IN 0x016C
#define CAN_ID_REF_CAN6_OUT 0x016D
#define CAN_ID_REF_CAN7_IN 0x016E
#define CAN_ID_REF_CAN7_OUT 0x016F

#define MRM_REF_CAN_SENSOR_COUNT 9 // Number of IR transistors in each device.

//CANBus commands
#define COMMAND_REF_CAN_MEASURE_ONCE_CENTER 0x04
#define COMMAND_REF_CAN_MEASURE_CONTINUOUS_CENTER 0x05
#define COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3 0x06
#define COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6 0x07
#define COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9 0x08
#define COMMAND_REF_CAN_CALIBRATE 0x09

class Mrm_ref_can : public SensorBoard
{
	uint16_t readings[MAX_SENSORS_BASE][MRM_REF_CAN_SENSOR_COUNT]; // Analog readings of all sensors
	
public:
	
	/** Constructor
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_ref_can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_ref_can();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Calibrate the array
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	*/
	void calibrate(uint8_t deviceNumber = 0);
	
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
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};




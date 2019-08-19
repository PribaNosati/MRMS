#pragma once
#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <CANBusBase.h>

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
#define MAX_MRM_REF_CAN 4 // Maximum number of Mrm-ref-can complete sensors. 
#define MRM_REF_CAN_SENSOR_COUNT 9 // Number of IR transistors in each sensor.

//CANBus commands
#define COMMAND_REPORT_ALIVE 0xFF

#define COMMAND_REFLECTANCE_ARRAY_MEASURE_ONCE_EACH 0x01
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_CONTINUOUS_EACH 0x02
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_STOP 0x03
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_ONCE_CENTER 0x04
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_CONTINUOUS_CENTER 0x05
#define COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_1_TO_3 0x06
#define COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_4_TO_6 0x07
#define COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_7_TO_9 0x08
#define COMMAND_REFLECTANCE_ARRAY_CALIBRATE 0x09

typedef bool(*BreakCondition)();

class Mrm_ref_can : CANBusBase
{
	bool aliveThis[MAX_MRM_REF_CAN]; // Responded to ping
	uint32_t idIn[MAX_MRM_REF_CAN];  // Inbound message id
	uint32_t idOut[MAX_MRM_REF_CAN]; // Outbound message id
	char nameThis[MAX_MRM_REF_CAN][10]; // Device's name
	int nextFree;
	BluetoothSerial* serial; // Additional serial port
	uint16_t readings[MAX_MRM_REF_CAN][MRM_REF_CAN_SENSOR_COUNT]; // Analog readings of all sensors
	
	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);
	
public:
	ESP32CANBus *esp32CANBus; // CANBus interface
	
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

	/** Did it respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	bool alive(uint8_t deviceNumber = 0) { return aliveThis[deviceNumber]; }

	/** Calibrate the array
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - calibrate all sensors.
	*/
	void calibrate(uint8_t sensorNumber = 0);
	
	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void continuousReadingStart(uint8_t sensorNumber = 0xFF);
	
	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void continuousReadingStop(uint8_t sensorNumber = 0xFF);
	
	/** Read CAN Bus message into local variables
	@param data - 8 bytes from CAN Bus message.
	*/
	void decodeMessage(uint8_t data[8], uint8_t sensorNumber = 0);

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	void devicesScan(bool verbose = true);

	/** Prints a frame
	@param msgId - CAN Bus message id
	@param dlc - data load byte count
	@param data - data
	@return - if true, found and printed
	*/
	bool framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]);
	
	/** Is the frame addressed to this device?
	@param canIdOut - CAN Bus id.
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - if true, it is
	*/
	bool isForMe(uint32_t canIdOut, uint8_t sensorNumber = 0);
	
	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t sensorNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};

//Declaration of error function. Definition is in Your code.
extern void error(char * message);



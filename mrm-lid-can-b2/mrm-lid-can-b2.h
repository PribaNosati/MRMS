#pragma once
#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <CANBusBase.h>

/**
Purpose: mrm-lid-can-b2 interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_VL53L1X0_IN 0x0150
#define CAN_ID_VL53L1X0_OUT 0x0151
#define CAN_ID_VL53L1X1_IN 0x0152
#define CAN_ID_VL53L1X1_OUT 0x0153
#define CAN_ID_VL53L1X2_IN 0x0154
#define CAN_ID_VL53L1X2_OUT 0x0155
#define CAN_ID_VL53L1X3_IN 0x0156
#define CAN_ID_VL53L1X3_OUT 0x0157
#define CAN_ID_VL53L1X4_IN 0x0158
#define CAN_ID_VL53L1X4_OUT 0x0159
#define MAX_MRM_LID_CAN_B2 12 // Maximum number of Mrm-lid-can-b2 complete sensors. 

//CANBus commands
#define COMMAND_REPORT_ALIVE 0xFF

#define COMMAND_LIDAR_MEASURE_ONCE 0x01
#define COMMAND_LIDAR_MEASURE_CONTINUOUS 0x02
#define COMMAND_LIDAR_MEASURE_STOP 0x03

typedef bool(*BreakCondition)();

class Mrm_lid_can_b2 : public CANBusBase
{
	bool aliveThis[MAX_MRM_LID_CAN_B2]; // Responded to ping
	uint32_t idIn[MAX_MRM_LID_CAN_B2];  // Inbound message id
	uint32_t idOut[MAX_MRM_LID_CAN_B2]; // Outbound message id
	char nameThis[MAX_MRM_LID_CAN_B2][10]; // Device's name
	int nextFree;
	BluetoothSerial * serial; // Additional serial port
	uint16_t readings[MAX_MRM_LID_CAN_B2]; // Analog readings of all sensors
	
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
	Mrm_lid_can_b2(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_lid_can_b2();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Did it respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	bool alive(uint8_t deviceNumber = 0) { return aliveThis[deviceNumber]; }
	
	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void continuousReadingStart(uint8_t sensorNumber = 0xFF);
	
	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void continuousReadingStop(uint8_t sensorNumber = 0xFF);
	
	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool decodeMessage(uint32_t canId, uint8_t data[8]);

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	void devicesScan(bool verbose = true);
	
	/** Is the frame addressed to this device?
	@param canIdOut - CAN Bus id.
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - if true, it is
	*/
	bool isForMe(uint32_t canIdOut, uint8_t sensorNumber = 0);

	/** Returns device's name
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - name
	*/
	String name(uint8_t sensorNumber);

	/** Prints a frame
	@param msgId - CAN Bus message id
	@param dlc - data load byte count
	@param data - data
	@return - if true, found and printed
	*/
	bool framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]);
	
	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t sensorNumber = 0);

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



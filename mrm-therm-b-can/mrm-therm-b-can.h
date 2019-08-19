#pragma once
#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <CANBusBase.h>

/**
Purpose: mrm-therm-b-can interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_THERM0_IN 0x0210 
#define CAN_ID_THERM0_OUT 0x0211
#define CAN_ID_THERM1_IN 0x0210 
#define CAN_ID_THERM1_OUT 0x0211
#define MAX_MRM_THERM_B_CAN 2 // Maximum number of mrm-ref-can boards. 

//CANBus commands
#define COMMAND_REPORT_ALIVE 0xFF

#define COMMAND_THERMO_MEASURE_ONCE 0x01
#define COMMAND_THERMO_MEASURE_CONTINUOUS 0x02
#define COMMAND_THERMO_MEASURE_STOP 0x03

typedef bool(*BreakCondition)();

class Mrm_therm_b_can : CANBusBase
{
	bool aliveThis[MAX_MRM_THERM_B_CAN]; // Responded to ping
	uint32_t idIn[MAX_MRM_THERM_B_CAN];  // Inbound message id
	uint32_t idOut[MAX_MRM_THERM_B_CAN]; // Outbound message id
	char nameThis[MAX_MRM_THERM_B_CAN][10]; // Device's name
	int nextFree;
	BluetoothSerial * serial; // Additional serial port
	int16_t readings[MAX_MRM_THERM_B_CAN]; // Highest temperature
	
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
	Mrm_therm_b_can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_therm_b_can();

	/** Add a mrm-therm-b-an board
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
	int16_t reading(uint8_t sensorNumber = 0);

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



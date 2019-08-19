#pragma once
#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <CANBusBase.h>

/**
Purpose: mrm-node interface to CANBus.
@author MRMS team
@version 0.1 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_NODE0_IN 0x170
#define CAN_ID_NODE0_OUT 0x171
#define CAN_ID_NODE1_IN 0x172
#define CAN_ID_NODE1_OUT 0x173
#define MAX_MRM_NODE 2 // Maximum number of mrm-node boards. 

//CANBus commands
#define COMMAND_REPORT_ALIVE 0xFF

typedef bool(*BreakCondition)();

class Mrm_node : CANBusBase
{
	bool aliveThis[MAX_MRM_NODE]; // Responded to ping
	uint32_t idIn[MAX_MRM_NODE];  // Inbound message id
	uint32_t idOut[MAX_MRM_NODE]; // Outbound message id
	char nameThis[MAX_MRM_NODE][10]; // Device's name
	int nextFree;
	BluetoothSerial * serial; // Additional serial port
	
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
	Mrm_node(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_node();

	/** Add a mrm-node board
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Did it respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	bool alive(uint8_t deviceNumber = 0) { return aliveThis[deviceNumber]; }

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	void devicesScan(bool verbose = true);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};

//Declaration of error function. Definition is in Your code.
extern void error(char * message);



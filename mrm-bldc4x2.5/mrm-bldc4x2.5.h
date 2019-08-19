#pragma once
#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <CANBusBase.h>

/**
Purpose: mrm-bldc4x2.5 interface to CANBus.
@author MRMS team
@version 0.0 2019-08-16
Licence: You can use this code any way you like.
*/

#define CAN_ID_BLDC4X2_5_MOTOR0_IN 0x0240
#define CAN_ID_BLDC4X2_5_MOTOR0_OUT 0x0241
#define CAN_ID_BLDC4X2_5_MOTOR1_IN 0x0242
#define CAN_ID_BLDC4X2_5_MOTOR1_OUT 0x0243
#define CAN_ID_BLDC4X2_5_MOTOR2_IN 0x0244
#define CAN_ID_BLDC4X2_5_MOTOR2_OUT 0x0245
#define CAN_ID_BLDC4X2_5_MOTOR3_IN 0x0246
#define CAN_ID_BLDC4X2_5_MOTOR3_OUT 0x0247
#define MAX_BLDC4X2_5 8 // Maximum number of motors attached to all mrm-bldc2x50 boards. 

//CANBus commands
#define COMMAND_REPORT_ALIVE 0xFF


typedef bool(*BreakCondition)();

class Mrm_bldc4x2_5 : public CANBusBase
{
	bool aliveThis[MAX_BLDC4X2_5]; // Responded to ping
	uint32_t idIn[MAX_BLDC4X2_5];  // Inbound message id
	uint32_t idOut[MAX_BLDC4X2_5]; // Outbound message id
	char nameThis[MAX_BLDC4X2_5][10]; // Device's name
	bool reversed[MAX_BLDC4X2_5]; // Change rotation
	bool left[MAX_BLDC4X2_5]; // Is on the left side
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
	Mrm_bldc4x2_5(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial = 0);

	~Mrm_bldc4x2_5();

	/** Add a motor attached to a mrm-bldc2x50 motor controller
	@param isReversed - changes rotation direction.
	@param isLeft - is on the left side
	@param deviceName - device's name
	*/
	void add(bool isReversed = false, bool isLeft = true, char * deviceName = "");

	/** Did it respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	bool alive(uint8_t deviceNumber = 0) { return aliveThis[deviceNumber]; }

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	void devicesScan(bool verbose = true);

	/** Start all motors
	@param leftSpeed
	@param right Speed
	*/
	void go(int8_t leftSpeed, int8_t rightSpeed);
	

	/** Returns device's name
	@param motorNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - name
	*/
	String name(uint8_t motorNumber);

	/** Motor speed
	@param motorNumber - motor's number
	@param speed - in range -127 to 127
	*/
	void setSpeed(uint8_t motorNumber, int8_t speed);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};

//Declaration of error function. Definition is in Your code.
extern void error(char * message);



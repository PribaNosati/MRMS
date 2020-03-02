#pragma once
#include "Arduino.h"
#include <mrm-action.h>
#include <mrm-board.h>

/**
Purpose: mrm-switch interface to CANBus.
@author MRMS team
@version 0.0 2019-12-07
Licence: You can use this code any way you like.
*/

#define MRM_SWITCHES_COUNT 2

class Mrm_switch : public SensorBoard
{
	std::vector<bool[MRM_SWITCHES_COUNT]>* lastOn;
	std::vector<ActionBase * [MRM_SWITCHES_COUNT]>* offOnAction;
	std::vector<uint8_t[MRM_SWITCHES_COUNT]>* pin;
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_switch(Robot* robot, uint8_t maxDevices = 1);

	~Mrm_switch();

	ActionBase* actionCheck();

	void actionSet(ActionBase* action, uint8_t switchNumber, uint8_t deviceNumber = 0);

	/** Add a mrm-switch board
	@param pin1 - ESP32 pin the first switch is connected to. Enter 0xFF if not in use.
	@param pin2 - ESP32 pin the second switch is connected to. Enter 0xFF if not in use.
	@param deviceName - device's name
	*/
	void add(uint8_t pin1 = 0xFF, uint8_t pin2 = 0xFF, char * deviceName = "");

	/** Read switch
	@param switchNumber
	@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - true if pressed, false otherwise
	*/
	bool read(uint8_t switchNumber, uint8_t deviceNumber = 0);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};



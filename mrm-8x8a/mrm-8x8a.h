#pragma once
#include "Arduino.h"
#include <mrm-action.h>
#include <mrm-board.h>

/**
Purpose: mrm-8x8a interface to CANBus.
@author MRMS team
@version 0.3 2019-09-07
Licence: You can use this code any way you like.
*/

#define CAN_ID_8x8A0_IN 0x200
#define CAN_ID_8x8A0_OUT 0x201
#define CAN_ID_8x8A1_IN 0x202
#define CAN_ID_8x8A1_OUT 0x203
#define CAN_ID_8x8A2_IN 0x204
#define CAN_ID_8x8A2_OUT 0x205
#define CAN_ID_8x8A3_IN 0x206
#define CAN_ID_8x8A3_OUT 0x207
#define CAN_ID_8x8A4_IN 0x208
#define CAN_ID_8x8A4_OUT 0x209
#define CAN_ID_8x8A5_IN 0x20A
#define CAN_ID_8x8A5_OUT 0x20B
#define CAN_ID_8x8A6_IN 0x20C
#define CAN_ID_8x8A6_OUT 0x20D
#define CAN_ID_8x8A7_IN 0x20E
#define CAN_ID_8x8A7_OUT 0x20F

//CANBus commands
#define COMMAND_8X8_DISPLAY 0x00
#define COMMAND_8X8_SWITCH_ON 0x01
#define COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION 0x02
#define COMMAND_8x8_TEST_CAN_BUS 0x03
#define COMMAND_8X8_BITMAP_DISPLAY_PART1 0x05
#define COMMAND_8X8_BITMAP_DISPLAY_PART2 0x06
#define COMMAND_8X8_BITMAP_DISPLAY_PART3 0x07
#define COMMAND_8X8_BITMAP_STORE_PART1 0x08
#define COMMAND_8X8_BITMAP_STORE_PART2 0x09
#define COMMAND_8X8_BITMAP_STORE_PART3 0x0A
#define COMMAND_8X8_BITMAP_STORED_DISPLAY 0x0B

#define MRM_8x8A_SWITCHES_COUNT 4

class Mrm_8x8a : public SensorBoard
{
	std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>* lastOn;
	std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>* on;
	std::vector<ActionBase *[MRM_8x8A_SWITCHES_COUNT]>* offOnAction;
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_8x8a(Robot* robot, uint8_t maxDevices = 1);

	~Mrm_8x8a();

	ActionBase* actionCheck();

	void actionSet(ActionBase* action, uint8_t switchNumber, uint8_t deviceNumber = 0);

	/** Add a mrm-8x8a board
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Display bitmap
	@param bitmapId - bitmap's id
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapDisplay(uint8_t bitmapId, uint8_t deviceNumber = 0);

	/** Display custom bitmap
	@param red - 8-byte array for red
	@param green - 8-byte array for green
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapCustomDisplay(uint8_t red[], uint8_t green[], uint8_t deviceNumber = 0);

	/** Store custom bitmap
	@param red - 8-byte array for red
	@param green - 8-byte array for green
	@param addres - address in display's RAM. 0 - 99.
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapCustomStore(uint8_t red[], uint8_t green[], uint8_t address, uint8_t deviceNumber = 0);

	/** Store custom bitmap
	@param addres - address in display's RAM. 0 - 99.
	@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void bitmapCustomStoredDisplay(uint8_t address, uint8_t deviceNumber = 0);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Read switch
	@param switchNumber
	@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - true if pressed, false otherwise
	*/
	bool switchRead(uint8_t switchNumber, uint8_t deviceNumber = 0);

	/**Test
	*/
	void test();

};



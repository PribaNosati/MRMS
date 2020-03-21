#pragma once
#include "Arduino.h"
#include "mrm-board.h"

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

#define CAN_ID_LID_CAN_B8_IN 0x0280
#define CAN_ID_LID_CAN_B8_OUT 0x0281
#define CAN_ID_LID_CAN_B9_IN 0x0282
#define CAN_ID_LID_CAN_B9_OUT 0x0283
#define CAN_ID_LID_CAN_B10_IN 0x0284
#define CAN_ID_LID_CAN_B10_OUT 0x0285
#define CAN_ID_LID_CAN_B11_IN 0x0286
#define CAN_ID_LID_CAN_B11_OUT 0x0287
#define CAN_ID_LID_CAN_B12_IN 0x0288
#define CAN_ID_LID_CAN_B12_OUT 0x0289
#define CAN_ID_LID_CAN_B13_IN 0x028A
#define CAN_ID_LID_CAN_B13_OUT 0x028B
#define CAN_ID_LID_CAN_B14_IN 0x028C
#define CAN_ID_LID_CAN_B14_OUT 0x028D
#define CAN_ID_LID_CAN_B15_IN 0x028E
#define CAN_ID_LID_CAN_B15_OUT 0x028F

//CANBus commands
#define COMMAND_LID_CAN_B_CALIBRATE 0x05


class Mrm_lid_can_b : public SensorBoard
{
	std::vector<uint16_t>* readings; // Analog readings of all sensors
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_lid_can_b(Robot* robot, uint8_t maxNumberOfBoards = 14);

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
	*/
	void test();

};



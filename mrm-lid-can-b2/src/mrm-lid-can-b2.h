#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-lid-can-b2 interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_LID_CAN_B2_0_IN 0x0150
#define CAN_ID_LID_CAN_B2_0_OUT 0x0151
#define CAN_ID_LID_CAN_B2_1_IN 0x0152
#define CAN_ID_LID_CAN_B2_1_OUT 0x0153
#define CAN_ID_LID_CAN_B2_2_IN 0x0154
#define CAN_ID_LID_CAN_B2_2_OUT 0x0155
#define CAN_ID_LID_CAN_B2_3_IN 0x0156
#define CAN_ID_LID_CAN_B2_3_OUT 0x0157
#define CAN_ID_LID_CAN_B2_4_IN 0x0158
#define CAN_ID_LID_CAN_B2_4_OUT 0x0159
#define CAN_ID_LID_CAN_B2_5_IN 0x015A
#define CAN_ID_LID_CAN_B2_5_OUT 0x015B
#define CAN_ID_LID_CAN_B2_6_IN 0x015C
#define CAN_ID_LID_CAN_B2_6_OUT 0x015D
#define CAN_ID_LID_CAN_B2_7_IN 0x015E
#define CAN_ID_LID_CAN_B2_7_OUT 0x015F

#define CAN_ID_LID_CAN_B2_8_IN 0x0270
#define CAN_ID_LID_CAN_B2_8_OUT 0x0271
#define CAN_ID_LID_CAN_B2_9_IN 0x0272
#define CAN_ID_LID_CAN_B2_9_OUT 0x0273
#define CAN_ID_LID_CAN_B2_10_IN 0x0274
#define CAN_ID_LID_CAN_B2_10_OUT 0x0275
#define CAN_ID_LID_CAN_B2_11_IN 0x0276
#define CAN_ID_LID_CAN_B2_11_OUT 0x0277
#define CAN_ID_LID_CAN_B2_12_IN 0x0278
#define CAN_ID_LID_CAN_B2_12_OUT 0x0279
#define CAN_ID_LID_CAN_B2_13_IN 0x027A
#define CAN_ID_LID_CAN_B2_13_OUT 0x027B
#define CAN_ID_LID_CAN_B2_14_IN 0x027C
#define CAN_ID_LID_CAN_B2_14_OUT 0x027D
#define CAN_ID_LID_CAN_B2_15_IN 0x027E
#define CAN_ID_LID_CAN_B2_15_OUT 0x027F

//CANBus commands
#define COMMAND_LID_CAN_B2_CALIBRATE 0x05
#define COMMAND_LID_CAN_B2_DISTANCE_MODE 0x50
#define COMMAND_LID_CAN_B2_TIMING_BUDGET 0x51
#define COMMAND_LID_CAN_B2_MEASUREMENT_TIME 0x52
#define COMMAND_LID_CAN_B2_ROI 0x53

#define MRM_LID_CAN_B2_INACTIVITY_ALLOWED_MS 10000

class Mrm_lid_can_b2 : public SensorBoard
{
	std::vector<uint16_t>* readings; // Analog readings of all sensors

	/** If sensor not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool started(uint8_t deviceNumber);
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_lid_can_b2(Robot* robot = NULL, uint8_t maxDevices = 12);

	~Mrm_lid_can_b2();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");
	
	/** Calibration, only once after production
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void calibration(uint8_t deviceNumber = 0);

	/** Reset sensor's non-volatile memory to defaults (distance mode, timing budget, region of interest, and measurement time, but leaves CAN Bus id intact
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - resets all.
	*/
	void defaults(uint8_t deviceNumber = 0xFF);

	/** Distance mode. Short mode has better ambient light immunity but the maximum distance is limited to 1.3 m. Long distance ranges up to 
		4 m but is less performant under ambient light. Stored in sensors non-volatile memory. Allow 50 ms for flash to be written.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param isShort - if not short, long.
	*/
	void distanceMode(uint8_t deviceNumber, bool isShort = true);

	/** Measurement time (IMP) in ms. IMP must be >= TB. Probably the best value is IMP = TB. Stored in sensors non-volatile memory.
	Allow 50 ms for flash to be written.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param ms - default 100.
	*/
	void measurementTime(uint8_t deviceNumber, uint16_t ms = 100);

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

	/** ROI, region of interest, a matrix from 4x4 up to 16x16 (x, y). Smaller x and y - smaller view angle. Stored in sensors non-volatile memory.
	Allow 50 ms for flash to be written.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param x - default 16.
	@param y - default 16.
	*/
	void roi(uint8_t deviceNumber, uint8_t x = 16, uint8_t y = 16);

	/**Test
	*/
	void test();

	/** Timing budget (TB) in ms. TB improves the measurement reliability but increases power consumption. Stored in sensors non-volatile memory.
			Set before measurement time as measurement time checks this value and returns error if not appropriate. Allow 50 ms for flash to be written.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param ms - (TB) in ms possible values [20, 50, 100, 200, 500]. Default 100.
	*/
	void timingBudget(uint8_t deviceNumber, uint16_t ms = 100);
};


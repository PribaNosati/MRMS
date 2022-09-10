#pragma once
#include "Arduino.h"
#include <mrm-board.h>

/**
Purpose: mrm-lid-can-b2 interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_LID_D_0_IN 0x0390
#define CAN_ID_LID_D_0_OUT 0x0391
#define CAN_ID_LID_D_1_IN 0x0392
#define CAN_ID_LID_D_1_OUT 0x0393
#define CAN_ID_LID_D_2_IN 0x0394
#define CAN_ID_LID_D_2_OUT 0x0395
#define CAN_ID_LID_D_3_IN 0x0396
#define CAN_ID_LID_D_3_OUT 0x0397
#define CAN_ID_LID_D_4_IN 0x0398
#define CAN_ID_LID_D_4_OUT 0x0399
#define CAN_ID_LID_D_5_IN 0x039A
#define CAN_ID_LID_D_5_OUT 0x039B
#define CAN_ID_LID_D_6_IN 0x039C
#define CAN_ID_LID_D_6_OUT 0x039D
#define CAN_ID_LID_D_7_IN 0x039E
#define CAN_ID_LID_D_7_OUT 0x039F

#define CAN_ID_LID_D_8_IN 0x0400
#define CAN_ID_LID_D_8_OUT 0x0401
#define CAN_ID_LID_D_9_IN 0x0402
#define CAN_ID_LID_D_9_OUT 0x0403
#define CAN_ID_LID_D_10_IN 0x0404
#define CAN_ID_LID_D_10_OUT 0x0405
#define CAN_ID_LID_D_11_IN 0x0406
#define CAN_ID_LID_D_11_OUT 0x0407
#define CAN_ID_LID_D_12_IN 0x0408
#define CAN_ID_LID_D_12_OUT 0x0409
#define CAN_ID_LID_D_13_IN 0x040A
#define CAN_ID_LID_D_13_OUT 0x040B
#define CAN_ID_LID_D_14_IN 0x040C
#define CAN_ID_LID_D_14_OUT 0x040D
#define CAN_ID_LID_D_15_IN 0x040E
#define CAN_ID_LID_D_15_OUT 0x040F

//CANBus commands
#define COMMAND_LID_D_RESOLUTION 0x05
#define COMMAND_LID_D_PNP_ENABLE 0x28
#define COMMAND_LID_D_PNP_DISABLE 0x29
#define COMMAND_LID_D_FREQUENCY 0x50

#define MRM_LID_D_INACTIVITY_ALLOWED_MS 10000

class Mrm_lid_d : public SensorBoard
{
	std::vector<uint8_t>* _resolution;
	std::vector<std::vector<uint16_t>>* readings; // Analog readings of all sensors

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
	Mrm_lid_d(Robot* robot = NULL, uint8_t maxDevices = 8);

	~Mrm_lid_d();

	/** Add a mrm-lid-d sensor
	@param deviceName - device's name
	@param resolution - 16 or 64, number of measuring dots
	*/
	void add(char * deviceName = (char*)"", uint8_t resolution = 16);
	
	/** Calibration, only once after production
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void calibration(uint8_t deviceNumber = 0);

	/** Reset sensor's non-volatile memory to defaults (distance mode, timing budget, region of interest, and measurement time, but leaves CAN Bus id intact
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - resets all.
	*/
	void defaults(uint8_t deviceNumber = 0xFF);

	/** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t distance(uint8_t deviceNumber, uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Minimum distance in mm. 
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - distance in mm
	*/
	uint16_t distanceShortest(uint8_t deviceNumber);

	/** Dot distance
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param x - x coordinate
	@param y - y coordinate
	@return - distance in mm
	*/
	uint16_t dot(uint8_t deviceNumber, uint8_t x, uint8_t y);

	/** Frequency.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param frequency - up to 60 for 4x4 and 15 for 8x8.
	*/
	void frequencySet(uint8_t deviceNumber, uint8_t frequency);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t length);

	/** Enable plug and play
	@param enable - enable or disable
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void pnpSet(bool enable = true, uint8_t deviceNumber = 0);

	/** Analog readings
	@param receiverNumberInSensor - always 0
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Resolution, 4x4 or 8x8.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param resolution - 16 or 64. Default 16.
	*/
	void resolutionSet(uint8_t deviceNumber, uint8_t resolution = 16);

	/**Test
	*/
	void test();
};


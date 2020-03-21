#pragma once
#include "Arduino.h"
#include <mrm-board.h>

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
#define CAN_ID_REF_CAN4_IN 0x0168
#define CAN_ID_REF_CAN4_OUT 0x0169
#define CAN_ID_REF_CAN5_IN 0x016A
#define CAN_ID_REF_CAN5_OUT 0x016B
#define CAN_ID_REF_CAN6_IN 0x016C
#define CAN_ID_REF_CAN6_OUT 0x016D
#define CAN_ID_REF_CAN7_IN 0x016E
#define CAN_ID_REF_CAN7_OUT 0x016F

#define MRM_REF_CAN_SENSOR_COUNT 9 // Number of IR transistors in each device.

//CANBus commands
#define COMMAND_REF_CAN_MEASURE_ONCE_CENTER 0x04
#define COMMAND_REF_CAN_MEASURE_CONTINUOUS_CENTER 0x05
#define COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3 0x06
#define COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6 0x07
#define COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9 0x08
#define COMMAND_REF_CAN_CALIBRATE 0x09
#define COMMAND_REF_CAN_CALIBRATION_DATA_DARK_1_TO_3 0x0A
#define COMMAND_REF_CAN_CALIBRATION_DATA_DARK_4_TO_6 0x0B
#define COMMAND_REF_CAN_CALIBRATION_DATA_DARK_7_TO_9 0x0C
#define COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_1_TO_3 0x0F
#define COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_4_TO_6 0x50
#define COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_7_TO_9 0x51
#define COMMAND_REF_CAN_CALIBRATION_DATA_REQUEST 0x0D
#define COMMAND_REF_CAN_SENDING_SENSORS_CENTER 0x0E
#define COMMAND_REPORT_ALIVE_QUEUELESS 0x0F // todo

class Mrm_ref_can : public SensorBoard
{
	std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>* calibrationDataDark; // 
	std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>* calibrationDataBright;
	std::vector<uint8_t>* dataFresh; // All the data refreshed, bitwise stored. 
									// Most significant bit 0: readings for transistors 1 - 3, 
									// bit 1: 4 - 6, 
									// bit 2: 7 - 9, 
									// bit 3: calibration data for transistors 1 - 3, 
									// bit 4: 4 - 6, 
									// bit 5: 7 - 9
	bool readingDigitalAndCenter = true; // Reading only center and transistors as bits. Otherwise reading all transistors as analog values.
	std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>* _reading; // Analog or digital readings of all sensors, depending on measuring mode.
	std::vector<uint16_t>* centerOfMeasurements; // Center of the dark sensors.

	/** Calibration data fresh?
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - yes or no
	*/
	bool dataCalibrationFreshAsk(uint8_t deviceNumber) { return ((*dataFresh)[deviceNumber] & 0b00011100) == 0b00011100; }

	/** All data fresh?
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - yes or no
	*/
	bool dataFreshAsk(uint8_t deviceNumber) { return (*dataFresh)[deviceNumber] == 0xFF; }

	/** Set calibration data freshness
	@param setToFresh - set value to be fresh. Otherwise set to not to be.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	*/
	void dataFreshCalibrationSet(bool setToFresh, uint8_t deviceNumber = 0);

	/** Set readings data freshness
	@param setToFresh - set value
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
	*/
	void dataFreshReadingsSet(bool setToFresh, uint8_t deviceNumber = 0);
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_ref_can(Robot* robot, uint8_t maxNumberOfBoards = 4);

	~Mrm_ref_can();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Any dark or bright
	@param dark - any dark? Otherwise, any bright?
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	bool any(bool dark = true, uint8_t deviceNumber = 0);

	/** Calibrate the array
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	*/
	void calibrate(uint8_t deviceNumber = 0);

	/** Get local calibration data
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param isDark - if true calibration for dark, otherwise for bright
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t calibrationDataGet(uint8_t receiverNumberInSensor, bool isDark, uint8_t deviceNumber = 0);

	/** Print all calibration data in a line
	*/
	void calibrationPrint();

	/** Request sensor to send calibration data
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@param waitForResult - Blocks program flow till results return.
	*/
	void calibrationDataRequest(uint8_t deviceNumber = 0, bool waitForResult = false);

	/** Center of measurements, like center of the line
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
	@return - 0 - nothing found. 1000 - 9000 for mrm-ref-can, 1000 - 8000 for ref-can8, 1000 - 6000 for mrm-ref-can6, and 1000 - 4000 for mrm-ref-can4. 
		1000 means center exactly under first sensor (the one closer to the biggest pin group).
	*/
	uint16_t center(uint8_t deviceNumber = 0) { return (*centerOfMeasurements)[deviceNumber]; }

	/** Dark?
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - yes or no.
	*/
	bool dark(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);
	
	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);
	
	/** Readings, can be analog or digital, depending on measuring mode
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	*/
	void test();

};




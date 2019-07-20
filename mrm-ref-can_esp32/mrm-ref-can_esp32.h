#pragma once
#include "Arduino.h"
#include <ESP32CANBus.h>

/**
Purpose: mrm-ref-can interface to CANBus.
@author MRMS team
@version 0.0 2019-07-20
Licence: You can use this code any way you like.
*/

#define MAX_MRM_REF_CAN 1 // Maximum number of Mrm-ref-can complete sensors. 
#define MRM_REF_CAN_SENSOR_COUNT 9 // Number of IR transistors in each sensor.

//CANBus commands
#define COMMAND_REPORT_ALIVE 0xFF

#define COMMAND_REFLECTANCE_ARRAY_MEASURE_ONCE_EACH 0x01
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_CONTINUOUS_EACH 0x02
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_STOP 0x03
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_ONCE_CENTER 0x04
#define COMMAND_REFLECTANCE_ARRAY_MEASURE_CONTINUOUS_CENTER 0x05
#define COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_1_TO_3 0x06
#define COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_4_TO_6 0x07
#define COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_7_TO_9 0x08
#define COMMAND_REFLECTANCE_ARRAY_CALIBRATE 0x09

typedef bool(*BreakCondition)();

class Mrm_ref_can
{
	uint32_t idIn[MAX_MRM_REF_CAN];  // Inbound message id
	uint32_t idOut[MAX_MRM_REF_CAN]; // Outbound message id
	int nextFree;
	HardwareSerial * serial; // Additional serial port
	ESP32CANBus *esp32CANBus; // CANBus interface
	uint16_t readings[MAX_MRM_REF_CAN][MRM_REF_CAN_SENSOR_COUNT]; // Analog readings of all sensors
	
	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);
	
public:
	/** Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_ref_can(HardwareSerial * hardwareSerial = 0);

	~Mrm_ref_can();

	/** Add a mrm-ref-can sensor
	@param addressIn - inbound message id
	@param addressOut - outbound message id
	*/
	void add(uint32_t addressIn, uint32_t addressOut);
	
	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void continuousReadingStart(uint8_t sensorNumber = 0xFF);
	
	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void continuousReadingStop(uint8_t sensorNumber = 0xFF);
	
	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t sensorNumber = 0);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

};

//Declaration of error function. Definition is in Your code.
void error(String message);



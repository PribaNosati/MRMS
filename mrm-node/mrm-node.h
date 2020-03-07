#pragma once
#include "Arduino.h"
#include <mrm-board.h>

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
#define CAN_ID_NODE2_IN 0x174
#define CAN_ID_NODE2_OUT 0x175
#define CAN_ID_NODE3_IN 0x176
#define CAN_ID_NODE3_OUT 0x177
#define CAN_ID_NODE4_IN 0x178
#define CAN_ID_NODE4_OUT 0x179
#define CAN_ID_NODE5_IN 0x17A
#define CAN_ID_NODE5_OUT 0x17B
#define CAN_ID_NODE6_IN 0x17C
#define CAN_ID_NODE6_OUT 0x17D
#define CAN_ID_NODE7_IN 0x17E
#define CAN_ID_NODE7_OUT 0x17F

#define MRM_NODE_ANALOG_COUNT 9
#define MRM_NODE_SWITCHES_COUNT 5
#define MRM_NODE_SERVO_COUNT 3

//CANBus commands
#define COMMAND_NODE_SENDING_SENSORS_1_TO_3 0x04
#define COMMAND_NODE_SENDING_SENSORS_4_TO_6 0x05
#define COMMAND_NODE_SENDING_SENSORS_7_TO_9 0x06
#define COMMAND_NODE_SWITCH_ON 0x07
#define COMMAND_NODE_SERVO_SET 0x08


class Mrm_node : public SensorBoard
{
	std::vector<uint16_t[MRM_NODE_ANALOG_COUNT]>* readings; // Analog readings of all sensors
	std::vector<bool[MRM_NODE_SWITCHES_COUNT]>* switches;
	std::vector<uint16_t[MRM_NODE_SERVO_COUNT]>* servoDegrees;// = { 0xFFFF, 0xFFFF, 0xFFFF };
	
public:

	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_node(Robot* robot, uint8_t maxNumberOfBoards = 2);

	~Mrm_node();

	/** Add a mrm-node board
	@param deviceName - device's name
	*/
	void add(char * deviceName = "");

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Analog readings
	@param receiverNumberInSensor - single IR transistor in mrm-ref-can
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t sensorNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Test servos
	*/
	void servoTest();

	/** Move servo
	@deviceNumber - mrm-node id
	@servoNumber - 0 - 2
	@degrees - 0 - 180 degrees
	*/
	void servoWrite(uint8_t servoNumber, uint16_t degrees, uint8_t deviceNumber = 0);

	/** Read digital
	@param switchNumber
	@deviceNumber - mrm-node id
	@return
	*/
	bool switchRead (uint8_t switchNumber, uint8_t deviceNumber = 0);

	/**Test
	*/
	void test();

};



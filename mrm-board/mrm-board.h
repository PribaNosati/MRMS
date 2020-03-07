#pragma once

#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <mrm-common.h>
#include <mrm-pid.h>
#include <vector>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_ONCE 0x11
#define COMMAND_SENSORS_MEASURE_STOP 0x12
#define COMMAND_SENSORS_MEASURE_SENDING 0x13
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION 0x14
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA 0x15
#define COMMAND_SENSORS_MEASURE_CALCULATED_SENDING 0x16
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2 0x17
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3 0x18
#define COMMAND_FIRMWARE_REQUEST 0x19
#define COMMAND_FIRMWARE_SENDING 0x1A
#define COMMAND_RESET 0x1B
#define COMMAND_SPEED_SET 0x20
#define COMMAND_SPEED_SET_REQUEST_NOTIFICATION 0x21
#define COMMAND_FPS_REQUEST 0x30
#define COMMAND_FPS_SENDING 0x31
#define COMMAND_ID_CHANGE_REQUEST 0x40
#define COMMAND_NOTIFICATION 0x41
#define COMMAND_ERROR 0xEE
#define COMMAND_REPORT_ALIVE 0xFF

#define MAX_MOTORS_IN_GROUP 4

#ifndef toRad
#define toRad(x) ((x) / 180.0 * PI) // Degrees to radians
#endif
#ifndef toDeg
#define toDeg(x) ((x) / PI * 180.0) // Radians to degrees
#endif

enum BoardType{MOTOR_BOARD, SENSOR_BOARD};

class Robot;

class Board{
protected:
	uint32_t _alive; // Responded to ping, maximum 32 devices of the same class, stored bitwise.
	uint8_t aliveTimeout = 4;//4
	char boardsName[12];
	BoardType boardTypeThis;
	uint8_t canData[8];
	uint8_t commandLastReceivedByTarget = 0xFE;
	uint8_t devicesOnABoard;
	//std::vector<bool>(maxNumberOfBoards * devicesOn1Board) deviceStarted; //todo - not to allow reading if the device not started.
	uint8_t errorCode = 0;
	uint8_t errorInDeviceNumber = 0;
	std::vector<uint16_t>* firmwareVersionLast;
	uint16_t fpsLast = 0xFFFF;
	std::vector<uint32_t>* idIn;  // Inbound message id
	std::vector<uint32_t>* idOut; // Outbound message id
	std::vector<uint32_t>* lastMessageReceivedMs;
	uint8_t maximumNumberOfBoards;
	uint8_t measuringMode = 0;
	uint8_t measuringModeLimit = 0;
	std::vector<char[10]>* nameThis;// Device's name
	int nextFree;
	Robot* robotContainer;

	/** Common part of message decoding
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void messageDecodeCommon(uint8_t deviceNumber = 0);

	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

public:
	
	/**
	@param robot - robot containing this board
	@param maxNumberOfBoards - maximum number of boards
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	*/
	Board(Robot* robot, uint8_t maxNumberOfBoards, uint8_t devicesOnABoard, char * boardName, BoardType boardTypeNow);

	/** Add a device
	@param deviceName
	@param canIn
	@param canOut
	*/
	void add(char* deviceName, uint16_t canIn, uint16_t canOut);

	/** Did it respond to last ping? If not, try another ping and see if it responds.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param checkAgainIfDead - try another ping
	@param errorIfNotAfterCheckingAgain - the robot will stop. Otherwise only warning displayed.
	@return - alive or not
	*/
	bool alive(uint8_t deviceNumber = 0, bool checkAgainIfDead = false, bool errorIfNotAfterCheckingAgain = false);

	/** Did any device respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	uint8_t aliveCount();

	/** Set aliveness
	@param yesOrNo
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void aliveSet(bool yesOrNo, uint8_t deviceNumber = 0);

	BoardType boardType(){ return boardTypeThis; }

	/** Count all the devices, alive or not
	@return - count
	*/
	uint8_t deadOrAliveCount();

	/** Number of devices in each group (board)
	@return - number of devices
	*/
	uint8_t devicesOnASingleBoard() { return this->devicesOnABoard; }

	/** Maximum number of devices in all groups (boards)
	@raturn - number of devices
	*/
	uint8_t devicesMaximumNumberInAllBoards() { return this->devicesOnABoard * this->maximumNumberOfBoards; }

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	@param timeout - ms to wait for response
	@return - alive count
	*/
	uint8_t devicesScan(bool verbose = true);

	/** Last error code
	@return - last error code from all devices of this kind
	*/
	uint8_t errorCodeLast() { return errorCode; }

	/** Device which caused last error
	@return - device number
	*/
	uint8_t errorWasInDeviceNumber() { return errorInDeviceNumber; }

	/** Firmware version
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - version
	*/
	uint16_t firmware(uint8_t deviceNumber = 0) { return (*firmwareVersionLast)[deviceNumber]; }

	/** Display firmware for all devices of a board
	*/
	void firmwareDisplay();

	/** Request firmware version
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void firmwareRequest(uint8_t deviceNumber = 0xFF);

	/** Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - FPS
	*/
	uint16_t fps(uint8_t deviceNumber = 0);

	/** Display FPS for all devices
	*/
	void fpsDisplay();

	/** Request Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void fpsRequest(uint8_t deviceNumber = 0xFF);

	/** Change CAN Bus id
	@param newDeviceNumber - new number
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void idChange(uint16_t newDeviceNumber, uint8_t deviceNumber = 0);

	/** Is the frame addressed to this device?
	@param canIdOut - CAN Bus id.
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - if true, it is
	*/
	bool isForMe(uint32_t canIdOut, uint8_t deviceNumber);

	/** Last message received
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - milliseconds
	*/
	uint32_t lastMessageMs(uint8_t deviceNumber = 0) { return (*lastMessageReceivedMs)[deviceNumber]; }

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	virtual bool messageDecode(uint32_t canId, uint8_t data[8]) = 0;

	/** Prints a frame
	@param frame - CAN Bus frame
	@return - if true, found and printed
	*/
	bool messagePrint(CAN_frame_t* frame);

	/** Prints a frame
	@param msgId - messageId
	@param dlc - data length
	@param data - payload
	@return - if true, found and printed
	*/
	bool messagePrint(uint32_t msgId, uint8_t dlc, uint8_t* data);

	/** Returns device's name
	@param deviceNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - name
	*/
	char * name(uint8_t deviceNumber);

	/** Returns device group's name
	@return - name
	*/
	char* name() {return boardsName;}

	/** Request notification
	@param commandRequestingNotification
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void notificationRequest(uint8_t commandRequestingNotification, uint8_t deviceNumber);

	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param measuringModeNow - Measuring mode id. Default 0.
	@param refreshMs - gap between 2 CAN Bus messages to refresh local Arduino copy of device's data. 0 - device's default.
	*/
	void start(uint8_t deviceNumber = 0xFF, uint8_t measuringModeNow = 0, uint16_t refreshMs = 0);

	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void stop(uint8_t deviceNumber = 0xFF);

	/**Test
	*/
	virtual void test() {}
};



class MotorBoard : public Board {
protected:
	std::vector<uint32_t>* encoderCount; // Encoder count
	std::vector<bool>* reversed; // Change rotation
public:

	/**
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param maxNumberOfBoards - maximum number of boards
	*/
	MotorBoard(Robot* robot, uint8_t devicesOnABoard, char * boardName, uint8_t maxNumberOfBoards);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Encoder readings
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - encoder value
	*/
	uint16_t reading(uint8_t deviceNumber);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/** Motor speed
	@param motorNumber - motor's number
	@param speed - in range -127 to 127
	*/
	void speedSet(uint8_t motorNumber, int8_t speed);

	void stop();

	/**Test
	*/
	void test();
};




class SensorBoard : public Board {
public:
	/**
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param devicesOnABoard - number of devices on each board
	@param boardName - board's name
	@param maxNumberOfBoards - maximum number of boards
	*/
	SensorBoard(Robot* robot, uint8_t devicesOnABoard, char* boardName, uint8_t maxNumberOfBoards);

	/** Starts periodical CANBus messages that will be refreshing values that mirror sensor's calculated values
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingCalculatedDataStart(uint8_t deviceNumber = 0xFF);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	virtual bool messageDecode(uint32_t canId, uint8_t data[8]){}
};

typedef void (*SpeedSetFunction)(uint8_t motorNumber, int8_t speed);

class MotorGroup {
protected:
	MotorBoard* motorBoard[MAX_MOTORS_IN_GROUP] = { NULL, NULL, NULL, NULL };
	uint8_t motorNumber[MAX_MOTORS_IN_GROUP];
public:
	MotorGroup();

	float angleNormalized(float angle);

	void stop();
};

class MotorGroupDifferential : public MotorGroup {
private:
	/** Check if speed is inside bounds
@param speed - speed to be checked
@return - speed inside bounds
*/
	int16_t checkBounds(int16_t speed);

public: 
	MotorGroupDifferential(MotorBoard* motorBoardForLeft1, uint8_t motorNumberForLeft1, MotorBoard* motorBoardForRight1, uint8_t motorNumberForRight1,
		MotorBoard* motorBoardForLeft2 = NULL, uint8_t motorNumberForLeft2 = 0, MotorBoard* motorBoardForRight2 = NULL, uint8_t motorNumberForRight2 = 0);

	/** Start all motors
	@param leftSpeed, in range -127 to 127
	@param right Speed, in range -127 to 127
	*/
	void go(int16_t leftSpeed = 0, int16_t rightSpeed = 0, int16_t lateralSpeedToRight = 0);
};

class MotorGroupStar : public MotorGroup {
public:
	MotorGroupStar(MotorBoard* motorBoardFor45Degrees, uint8_t motorNumberFor45Degrees, MotorBoard* motorBoardFor135Degrees, uint8_t motorNumberFor135Degrees,
		MotorBoard* motorBoardForMinus135Degrees, uint8_t motorNumberForMinus135Degrees, MotorBoard* motorBoardForMinus45Degrees, uint8_t motorNumberForMinus45Degrees);

	/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
	@param speed - 0 to 100.
	@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
	Values between -180 and 180.
	@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
	numbers because a value 100 turns on all the motors at maximal speed.
	@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
	*/
	void go(float speed, float angleDegrees = 0, float rotation = 0, uint8_t speedLimit = 127);
	
	/** Moves the robot in order to elinimate errors (for x and y directions).
	@param errorX - X axis error.
	@param errorY - Y axis error.
	@param headingToMaintain - Heading to maintain.
	@param verbose - print details
	*/
	void goToEliminateErrors(float errorX, float errorY, float headingToMaintain, Mrm_pid* pidXY, Mrm_pid* pidRotation, bool verbose = false);
};

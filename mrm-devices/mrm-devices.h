#pragma once

#include "Arduino.h"
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_ONCE 0x11
#define COMMAND_SENSORS_MEASURE_STOP 0x12
#define COMMAND_SENSORS_MEASURE_SENDING 0x13
#define COMMAND_SPEED_SET 0x20
#define COMMAND_FPS_REQUEST 0x30
#define COMMAND_FPS_SENDING 0x31
#define COMMAND_ID_CHANGE_REQUEST 0x40
#define COMMAND_LAST_REQUEST 0x41
#define COMMAND_LAST_SENDING 0x42
#define COMMAND_ERROR 0xEE
#define COMMAND_REPORT_ALIVE 0xFF

typedef bool(*BreakCondition)();

//Declaration of error function. Definition is in Your code.
extern void error(char* message);

class DeviceGroup{
protected:
	uint32_t aliveThis; // Responded to ping, maximum 32 devices of the same class
	uint8_t canData[8];
	uint8_t commandLastReceivedByTarget = 0xFE;
	uint8_t devicesInAGroup;
	uint8_t devicesMaxNumberInAllGroups;
	uint8_t errorCode = 0;
	uint8_t errorInDeviceNumber = 0;
	uint16_t fpsLast = 0xFFFF;
	char nameGroup[12];
	int nextFree;
	BluetoothSerial* serial; // Additional serial port

	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

public:

	ESP32CANBus* esp32CANBus; // CANBus interface
	
	DeviceGroup(ESP32CANBus* esp32CANBusSingleton, uint8_t devicesMaximumNumberInAllGroups, uint8_t devicesInAGroup, char * nameGroup);

	/** Did it respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	bool alive(uint8_t deviceNumber = 0);

	/** Did any device respond to last ping?
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	uint8_t aliveCount();

	/** Set aliveness
	@param yesOrNo
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void aliveSet(bool yesOrNo, uint8_t deviceNumber = 0);

	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	virtual void continuousReadingStart(uint8_t deviceNumber = 0xFF) = 0;

	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	virtual void continuousReadingStop(uint8_t deviceNumber = 0xFF) = 0;

	/** Count all the devices, alive or not
	@return - count
	*/
	uint8_t deadOrAliveCount();

	/** Number of devices in each group (board)
	@return - number of devices
	*/
	uint8_t devicesIn1Group() { return this->devicesInAGroup; }

	/** Maximum number of devices in all groups (boards)
	@raturn - number of devices
	*/
	uint8_t devicesMaximumNumberInAllGroups() { return this->devicesMaxNumberInAllGroups; }

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	virtual void devicesScan(bool verbose = true) = 0;

	/** Last error code
	@return - last error code from all devices of this kind
	*/
	uint8_t errorCodeLast() { return errorCode; }

	/** Device which caused last error
	@return - device number
	*/
	uint8_t errorWasInDeviceNumber() { return errorInDeviceNumber; }

	/** Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - FPS
	*/
	uint16_t fps(uint8_t deviceNumber = 0);

	/** Display FPS for all devices
	*/
	virtual void fpsDisplay() = 0;

	/** Request Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	virtual void fpsRequest(uint8_t deviceNumber = 0xFF) = 0;

	/** Prints a frame
	@param msgId - CAN Bus message id
	@param dlc - data load byte count
	@param data - data
	@return - if true, found and printed
	*/
	virtual bool framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]) = 0;

	/** Change CAN Bus id
	@param newDeviceNumber - new number
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	virtual void idChange(uint16_t newDeviceNumber, uint8_t deviceNumber = 0) = 0;

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	virtual bool messageDecode(uint32_t canId, uint8_t data[8]) = 0;

	/** Returns device's name
	@param deviceNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - name
	*/
	virtual char * name(uint8_t deviceNumber) = 0;

	/** Returns device group's name
	@return - name
	*/
	char* name() {return nameGroup;	}
};


#define MAX_MOTORS_BASE 8 // Maximum number of motors attached to all motor controllers of the same type

class MotorGroup : public DeviceGroup {
protected:
	uint32_t encoderCount[MAX_MOTORS_BASE]; // Encoder count
	uint32_t idIn[MAX_MOTORS_BASE];  // Inbound message id
	uint32_t idOut[MAX_MOTORS_BASE]; // Outbound message id
	bool left[MAX_MOTORS_BASE]; // Is on the left side
	char nameThis[MAX_MOTORS_BASE][10]; // Device's name
	bool reversed[MAX_MOTORS_BASE]; // Change rotation
public:

	MotorGroup(ESP32CANBus* esp32CANBusSingleton, uint8_t devicesInAGroup, char * nameGroup);

	/** add in base class*/
	void add(char* deviceName, uint16_t in0, uint16_t out0, uint16_t in1, uint16_t out1, uint16_t in2, uint16_t out2, uint16_t in3, uint16_t out3,
		uint16_t in4, uint16_t out4, uint16_t in5, uint16_t out5, uint16_t in6, uint16_t out6, uint16_t in7, uint16_t out7);

	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingStart(uint8_t deviceNumber = 0xFF);

	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingStop(uint8_t deviceNumber = 0xFF);

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	void devicesScan(bool verbose = true);

	/** Display FPS for all devices
	*/
	void fpsDisplay();

	/** Request Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void fpsRequest(uint8_t deviceNumber = 0xFF);

	/** Prints a frame
	@param msgId - CAN Bus message id
	@param dlc - data load byte count
	@param data - data
	@return - if true, found and printed
	*/
	bool framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]);

	/** Start all motors
	@param leftSpeed
	@param right Speed
	*/
	void go(int8_t leftSpeed, int8_t rightSpeed);

	/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
	@param speed - 0 to 100.
	@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
	Values between -180 and 180.
	@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
	numbers because a value 100 turns on all the motors at maximal speed.
	@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
	*/
	void goOmni(float speed, float angleDegrees, float rotation, uint8_t speedLimit = 127);

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

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8]);

	/** Returns device's name
	@param deviceNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - name
	*/
	char * name(uint8_t deviceNumber);

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

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	@param periodicFunction1 - A function that should be called while executing the test.
	@param periodicFunction2 - Another function that should be called while executing the test.
	*/
	void test(BreakCondition breakWhen = 0, void (*periodicFunction1)() = 0, void (*periodicFunction2)() = 0);
};



#define MAX_SENSORS_BASE 8 // Maximum number of device boards

class SensorBase : public DeviceGroup {
protected:
	uint32_t idIn[MAX_SENSORS_BASE];  // Inbound message id
	uint32_t idOut[MAX_SENSORS_BASE]; // Outbound message id
	char nameThis[MAX_SENSORS_BASE][10]; // Device's name
public:
	SensorBase(ESP32CANBus* esp32CANBusSingleton, uint8_t devicesInAGroup, char* nameGroup);

	/** add in base class*/
	void add(char* deviceName, uint16_t in0, uint16_t out0, uint16_t in1, uint16_t out1, uint16_t in2, uint16_t out2, uint16_t in3, uint16_t out3,
		uint16_t in4, uint16_t out4, uint16_t in5, uint16_t out5, uint16_t in6, uint16_t out6, uint16_t in7, uint16_t out7);

	/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingStart(uint8_t deviceNumber = 0xFF);

	/** Stops periodical CANBus messages that refresh values that can be read by reading()
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void continuousReadingStop(uint8_t deviceNumber = 0xFF);

	/** Ping devices and refresh alive array
	@param verbose - prints statuses
	*/
	void devicesScan(bool verbose = true);

	/** Display FPS for all devices
	*/
	void fpsDisplay();

	/** Request Frames Per Second
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
	*/
	void fpsRequest(uint8_t deviceNumber = 0xFF);

	/** Prints a frame
	@param msgId - CAN Bus message id
	@param dlc - data load byte count
	@param data - data
	@return - if true, found and printed
	*/
	bool framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]);

	/** Change CAN Bus id
	@param newDeviceNumber - new number
	@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void idChange(uint16_t newDeviceNumber, uint8_t deviceNumber = 0);

	/** Is the frame addressed to this device?
	@param canIdOut - CAN Bus id.
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - if true, it is
	*/
	bool isForMe(uint32_t canIdOut, uint8_t deviceNumber = 0);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@return - true if canId for this class
	*/
	virtual bool messageDecode(uint32_t canId, uint8_t data[8]){}

	/** Returns device's name
	@param deviceNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - name
	*/
	char * name(uint8_t deviceNumber);
};
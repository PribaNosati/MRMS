#include "mrm-board.h"
#include <mrm-pid.h>

#define REPORT_STRAY 0
#define REQUEST_NOTIFICATION 0

Board::Board(ESP32CANBus * esp32CANBusSingleton, uint8_t devicesMaximumNumberInAllGroups, uint8_t devicesInAGroup, char nameGroup[]) {
	esp32CANBus = esp32CANBusSingleton;
	this->devicesInAGroup = devicesInAGroup;
	this->devicesMaxNumberInAllGroups = devicesMaximumNumberInAllGroups;
	strcpy(this->nameGroup, nameGroup);
	aliveThis = 0;
	nextFree = 0;
}

/** Did it respond to last ping?
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
bool Board::alive(uint8_t deviceNumber) {
	if (deviceNumber > 31)
		error("Device number too big.");
	return (aliveThis >> deviceNumber) & 1;
}

/** Did any device respond to last ping?
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
uint8_t Board::aliveCount() {
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < nextFree; i++)
		if (alive(i))
			cnt++;
	return cnt;
}

/** Count all the devices, alive or not
@return - count
*/
uint8_t Board::deadOrAliveCount() { return nextFree; }

/** Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
@return - FPS
*/
uint16_t Board::fps(uint8_t deviceNumber) {
	return fpsLast;
}

/** Set aliveness
@param yesOrNo
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::aliveSet(bool yesOrNo, uint8_t deviceNumber) {
	if (deviceNumber > 31)
		error("Device number too big.");
	aliveThis = (aliveThis & ~(1 << deviceNumber)) | (yesOrNo << deviceNumber);
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Board::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Print to all serial ports, pointer to list
*/
void Board::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}



MotorBoard::MotorBoard(ESP32CANBus* esp32CANBusSingleton, uint8_t devicesInAGroup, char* nameGroup) : Board(esp32CANBusSingleton, MAX_MOTORS_BASE, devicesInAGroup, nameGroup) {
}

/** add in base class*/
void MotorBoard::add(char* deviceName, uint16_t in0, uint16_t out0, uint16_t in1, uint16_t out1, uint16_t in2, uint16_t out2, uint16_t in3, uint16_t out3,
	uint16_t in4, uint16_t out4, uint16_t in5, uint16_t out5, uint16_t in6, uint16_t out6, uint16_t in7, uint16_t out7) {
	if (nextFree >= MAX_SENSORS_BASE) {
		print("Too many devices: %s\n\r", deviceName);
		error("add");
	}
	if (deviceName != 0) {
		if (strlen(deviceName) > 9)
			error("Name too long");
		strcpy(nameThis[nextFree], deviceName);
	}
	switch (nextFree) {
	case 0:
		idIn[nextFree] = in0;
		idOut[nextFree] = out0;
		break;
	case 1:
		idIn[nextFree] = in1;
		idOut[nextFree] = out1;
		break;
	case 2:
		idIn[nextFree] = in2;
		idOut[nextFree] = out2;
		break;
	case 3:
		idIn[nextFree] = in3;
		idOut[nextFree] = out3;
		break;
	case 4:
		idIn[nextFree] = in4;
		idOut[nextFree] = out4;
		break;
	case 5:
		idIn[nextFree] = in5;
		idOut[nextFree] = out5;
		break;
	case 6:
		idIn[nextFree] = in6;
		idOut[nextFree] = out6;
		break;
	case 7:
		idIn[nextFree] = in7;
		idOut[nextFree] = out7;
		break;
	default:
		error("Too many devices\n\r");
	}
	nextFree++;
}


/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void MotorBoard::continuousReadingStart(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStart(i);
	else {
		if (alive(deviceNumber)) {
			print("Alive, start reading: %s\n\r", nameGroup);
#if REQUEST_NOTIFICATION
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, deviceNumber);
#else
			canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
			esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
#endif
		}
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void MotorBoard::continuousReadingStop(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStop(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
		}
	}
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
*/
void MotorBoard::devicesScan(bool verbose) {
	for (uint8_t i = 0; i < nextFree; i++) {
		canData[0] = COMMAND_REPORT_ALIVE;

		esp32CANBus->messageSend(idIn[i], 1, canData);
		//print("%s sent to 0x%4x\n\r", nameGroup, idIn[i]);
		uint32_t nowMs = millis();
		bool any = false;
		while (millis() - nowMs < 4 && !any)
			if (esp32CANBus->messageReceive()) {
				if (esp32CANBus->rx_frame->MsgID == idOut[i]) {
					if (verbose)
						print("%s found\n\r", nameThis[i]);
					any = true;
					aliveSet(true, i);
				}
#if REPORT_STRAY
				else
					if (verbose)
						print("stray id: %0x02X", (String)esp32CANBus->rx_frame->MsgID);
#endif
			}
		if (!any)
			aliveSet(false, i);
	}
}

/** Display FPS for all devices
*/
void MotorBoard::fpsDisplay() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (alive(deviceNumber))
			print("%s: %i FPS\n\r", nameThis[deviceNumber], fps(deviceNumber));
	}
}

/** Request Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.  0xFF - for all devices.
*/
void MotorBoard::fpsRequest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			fpsRequest(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_FPS_REQUEST;
			;
			esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
			fpsLast = 0;
		}
	}
}

/** Prints a frame
@param msgId - CAN Bus message id
@param dlc - data load byte count
@param data - data
@return - if true, found and printed
*/
bool MotorBoard::framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(msgId, deviceNumber)) {
			print("%s Id: 0x%04X", nameThis[deviceNumber], msgId);
			if (dlc > 0) {
				print(", data: ");
				for (uint8_t i = 0; i < dlc; i++)
					print("0x%02X ", data[i]);
			}
			print("\n\r");
			return true;
		}
	return false;
}

/** Change CAN Bus id
@param newId - CAN Bus id
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void MotorBoard::idChange(uint16_t newDeviceNumber, uint8_t deviceNumber) {
	canData[0] = COMMAND_ID_CHANGE_REQUEST;
	canData[1] = newDeviceNumber;
	esp32CANBus->messageSend(idIn[deviceNumber], 2, canData);
}

/** Is the frame addressed to this device?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it is
*/
bool MotorBoard::isForMe(uint32_t canIdOut, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree)
		error("Device doesn't exist");
	return canIdOut == idOut[deviceNumber];
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool MotorBoard::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++)
		if (isForMe(canId, motorNumber)) {
			switch (data[0]) {
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_SENSORS_MEASURE_SENDING: {
				uint32_t enc = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
				encoderCount[motorNumber] = enc;
				break;
			}
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = motorNumber;
				print("Error %i in %s.\n\r", errorCode, nameThis[motorNumber]);
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 200;
				errorInDeviceNumber = motorNumber;
				//error("MotDeco");
			}
			return true;
		}
	return false;
}

/** Returns device's name
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - name
*/
char* MotorBoard::name(uint8_t deviceNumber) {
	return nameThis[deviceNumber];
}

/** Request notification
@param commandRequestingNotification
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void MotorBoard::notificationRequest(uint8_t commandRequestingNotification, uint8_t deviceNumber) {
	uint8_t tries = 0;
	while (tries < 10) {
		tries++;
		canData[0] = commandRequestingNotification;
		esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
		while (tries != 0xFF && esp32CANBus->messageReceive()) {
			uint32_t id = esp32CANBus->rx_frame->MsgID;
			//print("RCVD id 0x%x, data: 0x%x\n\r", id, esp32CANBus->rx_frame->data.u8[0]);
			if (isForMe(id, deviceNumber) && esp32CANBus->rx_frame->data.u8[0] == COMMAND_NOTIFICATION) {
				tries = 0xFF;
				//print("OK...\n\r");
			}
		}
	}
	if (tries != 0xFF)
		error("Notification failed\n\r");
}

/** Encoder readings
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - encoder value
*/
uint16_t MotorBoard::reading(uint8_t deviceNumber) {
	if (deviceNumber > MAX_MOTORS_BASE)
		error("Device doesn't exist");
	return encoderCount[deviceNumber];
}

/** Print all readings in a line
*/
void MotorBoard::readingsPrint() {
	print("Encoders:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %4i", encoderCount[deviceNumber]);
}


/** Motor speed
@param motorNumber - motor's number
@param speed - in range -127 to 127
*/
void MotorBoard::speedSet(uint8_t motorNumber, int8_t speed) {
	if (motorNumber >= MAX_MOTORS_BASE) {
		print("Mot. %i", motorNumber);
		error(" doesn't exist");
	}

	if (reversed[motorNumber])
		speed = -speed;

	canData[0] = COMMAND_SPEED_SET;
	canData[1] = speed + 128;
	esp32CANBus->messageSend(idIn[motorNumber], 2, canData);


}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
@param periodicFunction1 - A function that should be called while executing the test.
@param periodicFunction2 - Another function that should be called while executing the test.
*/
void MotorBoard::test(BreakCondition breakWhen, void (*periodicFunction1)(), void (*periodicFunction2)())
{
	const uint16_t PAUSE_MS = 80;
	const uint16_t DISPLAY_PAUSE_MS = 300;
	const uint8_t STEP = 1;
	const uint8_t MAXIMUM_SPEED = 100; // Max. 127

	const int8_t startSpeed[3] = { 0, MAXIMUM_SPEED, -MAXIMUM_SPEED };
	const int8_t endSpeed[3] = { MAXIMUM_SPEED, -MAXIMUM_SPEED, 0 };
	const int8_t step[3] = { 1, -1, 1 };

	// Select motor
	int8_t selectedMotor = -2;
	uint32_t lastMs;
	while (selectedMotor < -1 || selectedMotor >= nextFree) {
		print("Enter motor number [0-%i] or wait for all\n\r", nextFree - 1);
		lastMs = millis();
		selectedMotor = -1;
		while (millis() - lastMs < 3000 && selectedMotor == -1)
			if (Serial.available()) {
				selectedMotor = Serial.read() - 48;
			}
		if (selectedMotor == -1)
			print("Test all\n\r");
		else if (selectedMotor >= 0 && selectedMotor < nextFree)
			print("\n\rTest motor %i\n\r", selectedMotor);
		else
			print("\n\rMotor %i doesn't exist\n\r", selectedMotor);
	}

	// Select speed
	int16_t selectedSpeed = 0;
	bool fixedSpeed = false;
	print("Enter speed [0-127] or wait for all\n\r");
	lastMs = millis();
	while (millis() - lastMs < 2000)
		if (Serial.available()) {
			selectedSpeed = selectedSpeed * 10 + (Serial.read() - 48);
			fixedSpeed = true;
			lastMs = millis();
		}
	if (fixedSpeed)
		print("\n\rSpeed %i\n\r", selectedSpeed);
	else
		print("All speeds\n\r");

	bool goOn = true;
	bool encodersStarted[4] = { false, false, false, false };
	lastMs = 0;
	while (goOn) {
		for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {

			if (selectedMotor != -1 && motorNumber != selectedMotor || !alive(motorNumber))
				continue;

			if (!encodersStarted[motorNumber]) {
				encodersStarted[motorNumber] = true;
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
				esp32CANBus->messageSend(idIn[motorNumber], 1, canData);
			}

			if (fixedSpeed) {
				speedSet(motorNumber, selectedSpeed);
				if ((*breakWhen)())
					goOn = false;
				delay(PAUSE_MS);
				continue;
			}

			for (uint8_t group = 0; group < 3; group++)
				for (int16_t speed = startSpeed[group];
				((step[group] > 0 && speed <= endSpeed[group]) || (step[group] < 0 && speed >= endSpeed[group])) && goOn;
					speed += step[group]) {
				//blink();
				if ((*breakWhen)())
					goOn = false;

				speedSet(motorNumber, speed);

				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("Mot. %i:%3i, en: %i\n\r", motorNumber, speed, encoderCount[motorNumber]);
					lastMs = millis();
				}
				uint32_t startMs = millis();
				while (millis() - startMs < PAUSE_MS) {
					(*periodicFunction1)();
					(*periodicFunction2)();
				}
			}

			speedSet(motorNumber, 0);
		}
	}

	// Stop all motors
	for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {
		speedSet(motorNumber, 0);

		if (encodersStarted[motorNumber]) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			esp32CANBus->messageSend(idIn[motorNumber], 1, canData);
		}
	}
}





SensorBoard::SensorBoard(ESP32CANBus* esp32CANBusSingleton, uint8_t devicesInAGroup, char nameGroup[]) : Board(esp32CANBusSingleton, MAX_SENSORS_BASE, devicesInAGroup, nameGroup) {
}

/** add in base class*/
void SensorBoard::add(char* deviceName, uint16_t in0, uint16_t out0, uint16_t in1, uint16_t out1, uint16_t in2, uint16_t out2, uint16_t in3, uint16_t out3, 
	uint16_t in4, uint16_t out4, uint16_t in5, uint16_t out5, uint16_t in6, uint16_t out6, uint16_t in7, uint16_t out7) {
	if (nextFree >= MAX_SENSORS_BASE) {
		print("Too many devices: %s\n\r", deviceName);
		error("add");
	}
	if (deviceName != 0) {
		if (strlen(deviceName) > 9)
			error("Name too long");
		strcpy(nameThis[nextFree], deviceName);
	}
	switch (nextFree) {
	case 0:
		idIn[nextFree] = in0;
		idOut[nextFree] = out0;
		break;
	case 1:
		idIn[nextFree] = in1;
		idOut[nextFree] = out1;
		break;
	case 2:
		idIn[nextFree] = in2;
		idOut[nextFree] = out2;
		break;
	case 3:
		idIn[nextFree] = in3;
		idOut[nextFree] = out3;
		break;
	case 4:
		idIn[nextFree] = in4;
		idOut[nextFree] = out4;
		break;
	case 5:
		idIn[nextFree] = in5;
		idOut[nextFree] = out5;
		break;
	case 6:
		idIn[nextFree] = in6;
		idOut[nextFree] = out6;
		break;
	case 7:
		idIn[nextFree] = in7;
		idOut[nextFree] = out7;
		break;
	default:
		error("Too many devices\n\r");
	}
	nextFree++;
}

/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::continuousReadingStart(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStart(i);
	else {
		if (alive(deviceNumber)) {
			print("Alive, start reading: %s\n\r", nameGroup);
#if REQUEST_NOTIFICATION
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, deviceNumber);
#else
			canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
			esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
#endif
		}
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::continuousReadingStop(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStop(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
		}
	}
}

/** Display FPS for all devices
*/
void SensorBoard::fpsDisplay() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (alive(deviceNumber))
			if (fpsLast == 0xFFFF)
				print("%s: no response\n\r", nameThis[deviceNumber]);
			else
				print("%s: %i FPS\n\r", nameThis[deviceNumber], fps(deviceNumber));
	}
}

/** Request Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.  0xFF - for all devices.
*/
void SensorBoard::fpsRequest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			fpsRequest(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_FPS_REQUEST;
			esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
			fpsLast = 0xFFFF;
		}
	}
}

/** Change CAN Bus id
@param newId - CAN Bus id
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::idChange(uint16_t newDeviceNumber, uint8_t deviceNumber) {
	canData[0] = COMMAND_ID_CHANGE_REQUEST;
	canData[1] = newDeviceNumber;
	esp32CANBus->messageSend(idIn[deviceNumber], 2, canData);
}

/** Is the frame addressed to this device?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it is
*/
bool SensorBoard::isForMe(uint32_t canIdOut, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree)
		error("Device doesn't exist");
	return canIdOut == idOut[deviceNumber];
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
*/
void SensorBoard::devicesScan(bool verbose) {
	for (uint8_t i = 0; i < nextFree; i++) {
		bool any = false;
#define SENSOR_BOARD_SCAN_TRIES 3
		for (uint8_t j = 0; j < SENSOR_BOARD_SCAN_TRIES && !any; j++) {
			canData[0] = COMMAND_REPORT_ALIVE;
			esp32CANBus->messageSend(idIn[i], 1, canData);
			//print("%s sent\n\r", nameGroup);
			uint32_t nowMs = millis();
			while (millis() - nowMs < 1 && !any) {
				//print("Sent at %i ms\n\r", millis());
				if (esp32CANBus->messageReceive()) {
					//print("Rcvd 0x%x\n\r", esp32CANBus->rx_frame->MsgID);
					if (esp32CANBus->rx_frame->MsgID == idOut[i]) {
						if (verbose)
							print("%s found\n\r", nameThis[i]);
						any = true;
					}
#if REPORT_STRAY
					else
						if (verbose)
							print("stray id: %0x02X", (String)esp32CANBus->rx_frame->MsgID);
#endif
				}
			}
		}
		aliveSet(any, i);
	}
	//print("%s OVER\n\r", nameGroup);
}

/** Prints a frame
@param msgId - CAN Bus message id
@param dlc - data load byte count
@param data - data
@return - if true, found and printed
*/
bool SensorBoard::framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(msgId, deviceNumber)) {
			print("%s Id: 0x%04X", nameThis[deviceNumber], msgId);
			if (dlc > 0) {
				print(", data: ");
				for (uint8_t i = 0; i < dlc; i++)
					print("0x%02X ", data[i]);
			}
			print("\n\r");
			return true;
		}
	return false;
}

/** Returns device's name
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - name
*/
char* SensorBoard::name(uint8_t deviceNumber) {
	return nameThis[deviceNumber];
}

/** Request notification
@param commandRequestingNotification
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::notificationRequest(uint8_t commandRequestingNotification, uint8_t deviceNumber) {
	uint8_t tries = 0;
	while (tries < 10) {
		tries++;
		canData[0] = commandRequestingNotification;
		esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
		while (tries != 0xFF && esp32CANBus->messageReceive()) {
			uint32_t id = esp32CANBus->rx_frame->MsgID;
			//print("RCVD id 0x%x, data: 0x%x\n\r", id, esp32CANBus->rx_frame->data.u8[0]);
			if (isForMe(id, deviceNumber) && esp32CANBus->rx_frame->data.u8[0] == COMMAND_NOTIFICATION) {
				tries = 0xFF;
				//print("OK...\n\r");
			}
		}
	}
	if (tries != 0xFF)
		error("Notification failed\n\r");
}

MotorGroup::MotorGroup(){
}

float MotorGroup::angleNormalized(float angle) {
	if (angle < -180)
		angle += 360;
	else if (angle > 180)
		angle -= 360;
	return angle;
}

void MotorGroup::stop() {
	for (uint8_t i = 0; i < MAX_MOTORS_IN_GROUP; i++)
		if (motorBoard[i] == NULL)
			break;
		else
			motorBoard[i]->speedSet(motorNumber[i], 0);
}

MotorGroupDifferential::MotorGroupDifferential(MotorBoard* motorBoardForLeft1, uint8_t motorNumberForLeft1, MotorBoard* motorBoardForRight1, uint8_t motorNumberForRight1,
	MotorBoard* motorBoardForLeft2, uint8_t motorNumberForLeft2, MotorBoard * motorBoardForRight2, uint8_t motorNumberForRight2) {
	motorBoard[0] = motorBoardForLeft1;
	motorNumber[0] = motorNumberForLeft1;
	motorBoard[1] = motorBoardForRight1;
	motorNumber[1] = motorNumberForRight1;
	motorBoard[2] = motorBoardForLeft2;
	motorNumber[2] = motorNumberForLeft2;
	motorBoard[3] = motorBoardForRight2;
	motorNumber[3] = motorNumberForRight2;
}

/** Start all motors
@param leftSpeed
@param right Speed
*/
void MotorGroupDifferential::go(int8_t leftSpeed, int8_t rightSpeed, int8_t lateralSpeedToRight) {
	if (motorBoard[0] != NULL) {
		motorBoard[0]->speedSet(motorNumber[0], leftSpeed - lateralSpeedToRight);
		motorBoard[1]->speedSet(motorNumber[1], rightSpeed + lateralSpeedToRight);
		motorBoard[2]->speedSet(motorNumber[2], leftSpeed - lateralSpeedToRight);
		motorBoard[3]->speedSet(motorNumber[3], rightSpeed + lateralSpeedToRight);
	}
}

MotorGroupStar::MotorGroupStar(MotorBoard* motorBoardFor45Degrees, uint8_t motorNumberFor45Degrees, MotorBoard* motorBoardFor135Degrees, uint8_t motorNumberFor135Degrees,
	MotorBoard* motorBoardForMinus135Degrees, uint8_t motorNumberForMinus135Degrees, MotorBoard* motorBoardForMinus45Degrees, uint8_t motorNumberForMinus45Degrees) {
	motorBoard[0] = motorBoardFor45Degrees;
	motorNumber[0] = motorNumberFor45Degrees;
	motorBoard[1] = motorBoardFor135Degrees;
	motorNumber[1] = motorNumberFor135Degrees;
	motorBoard[2] = motorBoardForMinus135Degrees;
	motorNumber[2] = motorNumberForMinus135Degrees;
	motorBoard[3] = motorBoardForMinus45Degrees;
	motorNumber[3] = motorNumberForMinus45Degrees;
}

/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
@param speed - 0 to 100.
@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
Values between -180 and 180.
@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
numbers because a value 100 turns on all the motors at maximal speed.
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void MotorGroupStar::go(float speed, float angleDegrees, float rotation, uint8_t speedLimit) {
	if (motorBoard[0] != NULL) {
		if (speedLimit == 0)
			stop();
		else {
			angleDegrees -= 45;
			float angleRadians = angleDegrees / 180 * 3.14;
			float si = sin(angleRadians);
			float co = cos(angleRadians);
			float xMinus135Deg = -speed * si + rotation; //135 degrees
			float x135Deg = -speed * co + rotation; //45 degrees
			float x45Deg = speed * si + rotation;  //-135 degrees
			float xMinus45Deg = speed * co + rotation;  //-45 degrees
			float speeds[4] = { x45Deg, x135Deg,xMinus135Deg,xMinus45Deg };

			if (speedLimit > 127)
				speedLimit = 127;
			float maxSpeed = abs(speeds[0]);
			for (int i = 1; i < 4; i++)
				if (abs(speeds[i]) > maxSpeed)
					maxSpeed = abs(speeds[i]);

			//Serial.print("Rot err: " + (String)rotation + " ");
			for (int i = 0; i < 4; i++)
				if (maxSpeed > speedLimit) {
					motorBoard[i]->speedSet(motorNumber[i], (int8_t)(speeds[i] / maxSpeed * speedLimit));
					//Serial.print("MAX ");
				}
				else {
					motorBoard[i]->speedSet(motorNumber[i], (int8_t)speeds[i]);
					//Serial.print((String)speeds[i] + " ");
				}
			//Serial.println();
		}
	}
}

/** Moves the robot in order to elinimate errors (for x and y directions).
@param errorX - X axis error.
@param errorY - Y axis error.
@param headingToMaintain - Heading to maintain.
@param verbose - print details
*/
void MotorGroupStar::goToEliminateErrors(float errorX, float errorY, float errorRotation, Mrm_pid *pidXY, Mrm_pid *pidRotation, bool verbose) {
	static uint32_t lastMs = 0;
	// Direction to maximally decrease error
	float heading;
	if (fabsf(errorX) > 0.001) // To avoid overflow.
		heading = toDeg(atan2f(errorX, errorY));
	else
		heading = errorY > 0 ? 0 : -180;
	//Serial.print(" Heading: " + (String)(int)heading + " ");

	// Speed to decrease direction error
	float speed;//todo - correct x - y directions according to heading error.
	errorRotation = angleNormalized(errorRotation);
	if (abs(errorRotation) < 20) // If a rotational error is quite big, correct without moving to x or y directions.
		speed = pidXY->calculate(fabsf(errorX) + fabsf(errorY), false); // PID sets the speed.
	else
		speed = 0;

	// Rotation to decrease z-axis rotation error
	float rotation = pidRotation->calculate(errorRotation);
	//Serial.println("\n\rABS: " + (String)(fabsf(errorX) + fabsf(errorY)) + "Speed " + (String)(int)speed + ", Rot: " + (String)(int)rotation);

	if (false && millis() - lastMs > 500) {
		lastMs = millis();
		//print(" Errors: (%i, %i) %i deg. Sp: %i, rot %i. \n\r", errorX, errorY, heading, speed, rotationToMaintainHeading(headingToMaintain));
	}
#define SPEED_LIMIT 30
	go(speed, heading, rotation, SPEED_LIMIT);
	if (verbose)
		Serial.println("Sp: " + (String)(int)speed + ", head: " + (String)(int)heading + ", rot: " + (int)rotation);
}
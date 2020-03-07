#include "mrm-board.h"
#include <mrm-pid.h>
#include "mrm-robot.h"

#define REPORT_STRAY 1
#define REQUEST_NOTIFICATION 0
#define ENABLE_DEVICE_DISCONNECT 1

/** Board is a single instance for all boards of the same type, not a single board (if there are more than 1 of the same type)! */

/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param maxNumberOfBoards - maximum number of boards
@param devicesOn1Board - number of devices on each board
@param boardName - board's name
*/
Board::Board(Robot* robot, uint8_t maxNumberOfBoards, uint8_t devicesOn1Board, char boardName[], BoardType boardTypeNow) {
	robotContainer = robot;
	idIn = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	idOut = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	nameThis = new std::vector<char[10]>(maxNumberOfBoards * devicesOn1Board);
	firmwareVersionLast = new std::vector<uint16_t>(maxNumberOfBoards);
	lastMessageReceivedMs = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	this->devicesOnABoard = devicesOn1Board;
	this->maximumNumberOfBoards = maxNumberOfBoards;
	strcpy(this->boardsName, boardName);
	_alive = 0;
	nextFree = 0;
	boardTypeThis = boardTypeNow;
	_message[28] = '\0';
}

/** Add a device
@param deviceName
@param canIn
@param canOut
*/
void Board::add(char* deviceName, uint16_t canIn, uint16_t canOut) {
	if (nextFree >= devicesMaximumNumberInAllBoards()) {
		sprintf(robotContainer->errorMessage, "Too many devices: %s", deviceName);
		return;
	}
	if (deviceName != 0) {
		if (strlen(deviceName) > 9) {
			sprintf(robotContainer->errorMessage, "Name too long: %s", deviceName);
			return;
		}
		strcpy((*nameThis)[nextFree], deviceName);
	}
	(*idIn)[nextFree] = canIn;
	(*idOut)[nextFree] = canOut;
	(*firmwareVersionLast)[nextFree] = 0;
	(*lastMessageReceivedMs)[nextFree] = 0;
	nextFree++;
}

/** Did it respond to last ping? If not, try another ping and see if it responds.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param checkAgainIfDead - try another ping
@param errorIfNotAfterCheckingAgain - the robot will stop. Otherwise only warning displayed.
@return - alive or not
*/
bool Board::alive(uint8_t deviceNumber, bool checkAgainIfDead, bool errorIfNotAfterCheckingAgain) {
	if (deviceNumber > 31) {
		strcpy(robotContainer->errorMessage, "Device number too big.");
		return false;
	}

	if ((_alive >> deviceNumber) & 1)
		return true;
	else if (checkAgainIfDead) {
		devicesScan(false);
		if ((_alive >> deviceNumber) & 1)
			return true;
		else {
			if (errorIfNotAfterCheckingAgain)
				sprintf(robotContainer->errorMessage, "%s no. %i dead", name(), deviceNumber);
			else
				print("%s %i dead\n\r", name(), deviceNumber);
			return false;
		}
	}
	else
		return false;
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

/** Set aliveness
@param yesOrNo
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::aliveSet(bool yesOrNo, uint8_t deviceNumber) {
	if (deviceNumber > 31) {
		strcpy(robotContainer->errorMessage, "Device number too big.");
		return;
	}
	_alive = (_alive & ~(1 << deviceNumber)) | (yesOrNo << deviceNumber);
}

/** Count all the devices, alive or not
@return - count
*/
uint8_t Board::deadOrAliveCount() { return nextFree; }

/** Ping devices and refresh alive array
@param verbose - prints statuses
@return - alive count
*/
uint8_t Board::devicesScan(bool verbose) {
	uint8_t count = 0;
	for (uint8_t i = 0; i < nextFree; i++) {
		bool any = false;
#define BOARD_SCAN_TRIES 1
		for (uint8_t j = 0; j < BOARD_SCAN_TRIES && !any; j++) {
			canData[0] = COMMAND_REPORT_ALIVE;
			robotContainer->esp32CANBus->messageSend((*idIn)[i], 1, canData);
			//print("%s sent to 0x%x\n\r", (*nameThis)[i], (*idIn)[i]);
			uint32_t nowMs = millis();
			while (millis() - nowMs < aliveTimeout && !any) { // 1 is not enough, stray msgs reported
				//print("Sent at %i ms\n\r", millis());
				if (robotContainer->esp32CANBus->messageReceive()) {
					//print("Rcvd 0x%x, expected: 0x%x\n\r", esp32CANBus->rx_frame->MsgID, (*idOut)[i]);
					if (robotContainer->esp32CANBus->rx_frame->MsgID == (*idOut)[i]) {
						if (verbose)
							print("%s found\n\r", (*nameThis)[i]);
						any = true;
						count++;
					}
#if REPORT_STRAY
					else
						if (verbose) {
							print("Stray msg: ");
							messagePrint(robotContainer->esp32CANBus->rx_frame);
							print("\n\r");
							//print("stray id from: 0x%02X\n\r", robotContainer->esp32CANBus->rx_frame->MsgID);
						}
#endif
				}
			}
		}
#if ENABLE_DEVICE_DISCONNECT
		aliveSet(any, i);
#else
		if (any)
			aliveSet(true, i);
#endif
	}
	//print("%s OVER\n\r", nameGroup);
	return count;
}

/** Display firmware for all devices of a board
*/
void Board::firmwareDisplay() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (alive(deviceNumber))
			if ((*firmwareVersionLast)[deviceNumber] == 0xFFFF)
				print("%s: no response\n\r", (*nameThis)[deviceNumber]);
			else
				print("%s: ver. %i \n\r", (*nameThis)[deviceNumber], firmware(deviceNumber));
	}
}

/** Request firmware version
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::firmwareRequest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			firmwareRequest(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_FIRMWARE_REQUEST;
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
			(*firmwareVersionLast)[deviceNumber] = 0;
		}
	}
}

/** Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - for all devices.
@return - FPS
*/
uint16_t Board::fps(uint8_t deviceNumber) {
	return fpsLast;
}

/** Display FPS for all devices
*/
void Board::fpsDisplay() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (alive(deviceNumber))
			if (fpsLast == 0xFFFF)
				print("%s: no response\n\r", (*nameThis)[deviceNumber]);
			else
				print("%s: %i FPS\n\r", (*nameThis)[deviceNumber], fps(deviceNumber));
	}
}

/** Request Frames Per Second
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.  0xFF - for all devices.
*/
void Board::fpsRequest(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			fpsRequest(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_FPS_REQUEST;
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
			fpsLast = 0xFFFF;
		}
	}
}

/** Change CAN Bus id
@param newId - CAN Bus id
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::idChange(uint16_t newDeviceNumber, uint8_t deviceNumber) {
	canData[0] = COMMAND_ID_CHANGE_REQUEST;
	canData[1] = newDeviceNumber;
	robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 2, canData);
}

/** Is the frame addressed to this device?
@param canIdOut - CAN Bus id.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the deviceNumber, starting with 0.
@return - if true, it is
*/
bool Board::isForMe(uint32_t canIdOut, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		strcpy(robotContainer->errorMessage, "Board doesn't exist");
		return false;
	}
	return canIdOut == (*idOut)[deviceNumber];
}

/** Common part of message decoding
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - command found
*/
bool Board::messageDecodeCommon(uint32_t canId, uint8_t data[8], uint8_t deviceNumber) {
	(*lastMessageReceivedMs)[deviceNumber] = millis();
	bool found = true;
	uint8_t command = data[0];
	switch (command) {
	case COMMAND_ERROR:
		errorCode = data[1];
		errorInDeviceNumber = deviceNumber;
		print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
		break;
	case COMMAND_FIRMWARE_SENDING:
		(*firmwareVersionLast)[deviceNumber] = (data[2] << 8) | data[1];
		break;
	case COMMAND_FPS_SENDING:
		fpsLast = (data[2] << 8) | data[1];
		break;
	case COMMAND_MESSAGE_SENDING_1:
		print("From %s \n\r", (*nameThis)[deviceNumber]);
		for (uint8_t i = 0; i < 7; i++) {
			print("%i ", data[i + i]);
			_message[i] = data[i + i];
		}
		break;
	case COMMAND_MESSAGE_SENDING_2:
		for (uint8_t i = 0; i < 7; i++)
			_message[7 + i] = data[i + i];
		break;
	case COMMAND_MESSAGE_SENDING_3:
		for (uint8_t i = 0; i < 7; i++)
			_message[14 + i] = data[i + i];
		break;
	case COMMAND_MESSAGE_SENDING_4:
		for (uint8_t i = 0; i < 7; i++)
			_message[21 + i] = data[i + i];
		print("Message from %s: %s\n\r", (*nameThis)[deviceNumber], _message);
		break;
	case COMMAND_NOTIFICATION:
		break;
	case COMMAND_REPORT_ALIVE:
		break;
	default:
		found = false;
	}
	return found;
}


/** Prints a frame
@param frame - CAN Bus frame
@return - if true, found and printed
*/
bool Board::messagePrint(CAN_frame_t* frame) {
	CAN_FIR_t fir = robotContainer->esp32CANBus->rx_frame->FIR;
	uint32_t msgId = robotContainer->esp32CANBus->rx_frame->MsgID;
	uint8_t dlc = fir.B.DLC;
	return messagePrint(msgId, dlc, frame->data.u8);
}

/** Prints a frame
@param msgId - messageId
@param dlc - data length
@param data - payload
@return - if true, found and printed
*/
bool Board::messagePrint(uint32_t msgId, uint8_t dlc, uint8_t* data) {
	bool found = false;
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(msgId, deviceNumber)) {
			print("Id:%s (0x%02X)", (*nameThis)[deviceNumber], msgId);
			found = true;
		}
	if (!found)
		print("Id:0x%02X", msgId);
	for (uint8_t i = 0; i < dlc; i++) {
		if (i == 0)
			print(" data:");
		print(" %02X", data[i]);
	}
	return found;
}

/** Returns device's name
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - name
*/
char* Board::name(uint8_t deviceNumber) {
	return (*nameThis)[deviceNumber];
}

/** Request notification
@param commandRequestingNotification
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::notificationRequest(uint8_t commandRequestingNotification, uint8_t deviceNumber) {
	uint8_t tries = 0;
	while (tries < 10) {
		tries++;
		canData[0] = commandRequestingNotification;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
		while (tries != 0xFF && robotContainer->esp32CANBus->messageReceive()) {
			uint32_t id = robotContainer->esp32CANBus->rx_frame->MsgID;
			//print("RCVD id 0x%x, data: 0x%x\n\r", id, esp32CANBus->rx_frame->data.u8[0]);
			if (isForMe(id, deviceNumber) && robotContainer->esp32CANBus->rx_frame->data.u8[0] == COMMAND_NOTIFICATION) {
				tries = 0xFF;
				//print("OK...\n\r");
			}
		}
	}
	if (tries != 0xFF)
		strcpy(robotContainer->errorMessage, "Notification failed");
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


/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param measuringModeNow - Measuring mode id. Default 0.
@param refreshMs - gap between 2 CAN Bus messages to refresh local Arduino copy of device's data. 0 - device's default.
*/
void Board::start(uint8_t deviceNumber, uint8_t measuringModeNow, uint16_t refreshMs) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			start(i, measuringModeNow, refreshMs);
	else {
		if (alive(deviceNumber)) {
			print("Alive, start reading: %s\n\r", boardsName);
#if REQUEST_NOTIFICATION
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, deviceNumber);
#else
			if (measuringModeNow == 0 || measuringModeLimit == 0)
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
			else if (measuringModeNow == 1 || measuringModeLimit >= 1)
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2;
			else
				canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3;
			measuringMode = measuringModeNow;
			if (refreshMs != 0) {
				canData[1] = refreshMs & 0xFF;
				canData[2] = (refreshMs >> 8) & 0xFF;
			}
			//messagePrint((*idIn)[deviceNumber], refreshMs == 0 ? 1 : 3, canData);
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], refreshMs == 0 ? 1 : 3, canData);
#endif
		}
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::stop(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			stop(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
		}
	}
}

/** Print to all serial ports, pointer to list
*/
void Board::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (robotContainer->serialBT() != 0)
		robotContainer->serialBT()->print(buffer);
}


/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param devicesOnABoard - number of devices on each board
@param boardName - board's name
@param maxNumberOfBoards - maximum number of boards
*/
MotorBoard::MotorBoard(Robot* robot, uint8_t devicesOnABoard, char* boardName, uint8_t maxNumberOfBoards) :
	Board(robot, maxNumberOfBoards, devicesOnABoard, boardName, MOTOR_BOARD) {
	encoderCount = new std::vector<uint32_t>(devicesOnABoard * maxNumberOfBoards);
	reversed = new std::vector<bool>(devicesOnABoard * maxNumberOfBoards);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool MotorBoard::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint32_t enc = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
					(*encoderCount)[deviceNumber] = enc;
					break;
				}
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data);
					print("\n\r");
					errorCode = 200;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}


/** Encoder readings
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - encoder value
*/
uint16_t MotorBoard::reading(uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		strcpy(robotContainer->errorMessage, "MotorBoard doesn't exist");
		return 0;
	}
	return (*encoderCount)[deviceNumber];
}

/** Print all readings in a line
*/
void MotorBoard::readingsPrint() {
	print("Encoders:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %4i", (*encoderCount)[deviceNumber]);
}


/** Motor speed
@param motorNumber - motor's number
@param speed - in range -127 to 127
*/
void MotorBoard::speedSet(uint8_t motorNumber, int8_t speed) {
	if (motorNumber >= nextFree) {
		sprintf(robotContainer->errorMessage, "Mot. %i doesn't exist", motorNumber);
		return;
	}

	if ((*reversed)[motorNumber])
		speed = -speed;

	canData[0] = COMMAND_SPEED_SET;
	canData[1] = speed + 128;
	robotContainer->esp32CANBus->messageSend((*idIn)[motorNumber], 2, canData);


}

void MotorBoard::stop() {
	for (uint8_t i = 0; i < nextFree; i++)
		speedSet(i, 0);
}

/**Test
*/
void MotorBoard::test()
{
	const uint16_t PAUSE_MS = 20;
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
				robotContainer->esp32CANBus->messageSend((*idIn)[motorNumber], 1, canData);
			}

			if (fixedSpeed) {
				speedSet(motorNumber, selectedSpeed);
				if (robotContainer->userBreak())
					goOn = false;
				robotContainer->delayMs(PAUSE_MS);
				continue;
			}

			for (uint8_t group = 0; group < 3; group++)
				for (int16_t speed = startSpeed[group];
				((step[group] > 0 && speed <= endSpeed[group]) || (step[group] < 0 && speed >= endSpeed[group])) && goOn;
					speed += step[group]) {
				//blink();
				if (robotContainer->userBreak())
					goOn = false;

				speedSet(motorNumber, speed);

				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("Mot. %i:%3i, en: %i\n\r", motorNumber, speed, (*encoderCount)[motorNumber]);
					lastMs = millis();
				}
				robotContainer->delayMs(PAUSE_MS);
			}

			speedSet(motorNumber, 0);
		}
	}

	// Stop all motors
	for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {
		speedSet(motorNumber, 0);

		if (encodersStarted[motorNumber]) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			robotContainer->esp32CANBus->messageSend((*idIn)[motorNumber], 1, canData);
		}
	}
}




/**
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param devicesOnABoard - number of devices on each board
@param boardName - board's name
@param maxNumberOfBoards - maximum number of boards
*/
SensorBoard::SensorBoard(Robot* robot, uint8_t devicesOnABoard, char boardName[], uint8_t maxNumberOfBoards) : 
	Board(robot, maxNumberOfBoards, devicesOnABoard, boardName, SENSOR_BOARD) {
}

/** Starts periodical CANBus messages that will be refreshing values that mirror sensor's calculated values
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void SensorBoard::continuousReadingCalculatedDataStart(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingCalculatedDataStart(i);
	else {
		if (alive(deviceNumber)) {
			print("Alive, start reading: %s\n\r", boardsName);
#if REQUEST_NOTIFICATION // todo
			notificationRequest(COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION, deviceNumber);
#else
			canData[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA;
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
			//print("Sent to 0x%x\n\r, ", (*idIn)[deviceNumber]);
#endif
		}
	}
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

/** Check if speed is inside bounds
@param speed - speed to be checked
@return - speed inside bounds
*/
int16_t MotorGroupDifferential::checkBounds(int16_t speed) {
	if (speed < -127)
		return -127;
	else if (speed > 127)
		return 127;
	else 
		return speed;
}

/** Start all motors
@param leftSpeed, in range -127 to 127
@param right Speed, in range -127 to 127
*/
void MotorGroupDifferential::go(int16_t leftSpeed, int16_t rightSpeed, int16_t lateralSpeedToRight) {
	if (motorBoard[0] != NULL) {
		motorBoard[0]->speedSet(motorNumber[0], checkBounds(leftSpeed - lateralSpeedToRight));
		motorBoard[1]->speedSet(motorNumber[1], checkBounds(-rightSpeed + lateralSpeedToRight));
		motorBoard[2]->speedSet(motorNumber[2], checkBounds(leftSpeed - lateralSpeedToRight));
		motorBoard[3]->speedSet(motorNumber[3], checkBounds(-rightSpeed + lateralSpeedToRight));
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
			angleDegrees += 135;// -= 45;
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

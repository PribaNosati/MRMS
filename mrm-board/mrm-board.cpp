#include "mrm-board.h"
#include <mrm-pid.h>

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
Board::Board(Robot* robot, uint8_t maxNumberOfBoards, uint8_t devicesOn1Board, char boardName[], BoardType boardTypeThis) {
	robotContainer = robot;
	idIn = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	idOut = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	nameThis = new std::vector<char[10]>(maxNumberOfBoards * devicesOn1Board);
	firmwareVersionLast = new std::vector<uint16_t>(maxNumberOfBoards);
	lastMessageReceivedMs = new std::vector<uint32_t>(maxNumberOfBoards * devicesOn1Board);
	this->devicesOnABoard = devicesOn1Board;
	this->maximumNumberOfBoards = maxNumberOfBoards;
	strcpy(this->boardsName, boardName);
	aliveThis = 0;
	nextFree = 0;
	boardTypeThis = boardTypeThis;
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

/** Did it respond to last ping?
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
bool Board::alive(uint8_t deviceNumber) {
	if (deviceNumber > 31) {
		strcpy(robotContainer->errorMessage, "Device number too big.");
		return 0;
	}
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

/** Set aliveness
@param yesOrNo
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::aliveSet(bool yesOrNo, uint8_t deviceNumber) {
	if (deviceNumber > 31) {
		strcpy(robotContainer->errorMessage, "Device number too big.");
		return;
	}
	aliveThis = (aliveThis & ~(1 << deviceNumber)) | (yesOrNo << deviceNumber);
}

/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param measuringModeNow - Measuring mode id.
*/
void Board::continuousReadingStart(uint8_t deviceNumber, uint8_t measuringModeNow) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStart(i, measuringModeNow);
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
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
#endif
		}
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::continuousReadingStop(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStop(i);
	else {
		if (alive(deviceNumber)) {
			canData[0] = COMMAND_SENSORS_MEASURE_STOP;
			robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
		}
	}
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
						if (verbose)
							print("stray id: %0x02X\n\r", (String)robotContainer->esp32CANBus->rx_frame->MsgID);
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

/** Prints a frame
@param msgId - CAN Bus message id
@param dlc - data load byte count
@param data - data
@return - if true, found and printed
*/
bool Board::framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(msgId, deviceNumber)) {
			print("%s Id: 0x%04X", (*nameThis)[deviceNumber], msgId);
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
@param deviceNumber - Devices's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Board::messageDecodeCommon(uint8_t deviceNumber) {
	(*lastMessageReceivedMs)[deviceNumber] = millis();
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
			messageDecodeCommon(deviceNumber);
			switch (data[0]) {
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_SENSORS_MEASURE_SENDING: {
				uint32_t enc = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
				(*encoderCount)[deviceNumber] = enc;
				break;
			}
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 200;
				errorInDeviceNumber = deviceNumber;
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
				delay(PAUSE_MS);
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
				uint32_t startMs = millis();
				while (millis() - startMs < PAUSE_MS) {
					robotContainer->blink();
					robotContainer->messagesReceive();
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

void doNothing(){}

/**
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Robot::Robot(BluetoothSerial* hardwareSerial) {
	esp32CANBus = new ESP32CANBus();
	serial = hardwareSerial;
	commandCurrent = NULL;
	//commandCurrent = &commandMenuPrint; //todo
	commandPrevious = commandCurrent;
	for (uint8_t i = 0; i < COMMANDS_LIMIT; i++)
		commands[i] = NULL;
	menuAdd(&commandDoNothing, 0, "Do nothing", &doNothing, 0);//UNKNOWN_ROBOT -> display for all robots
}

void Robot::add(Board* aBoard) {
	if (nextFreeBoardSlot > MRM_BOARD_COUNT - 1) {
		strcpy(errorMessage, "Too many boards");
		return;
	}
	board[nextFreeBoardSlot++] = aBoard;
}

#define LED_OK 2 // Pin number, hardware defined
void Robot::blink() {
	const uint16_t onMs = 100;
	const uint16_t offMs = 1000;
	uint8_t repeatOnTimes;
	static uint32_t lastBlinkMs = 0;
	static uint8_t isOn = 0;
	static uint8_t pass = 0;

	if (strcmp(errorMessage, "") == 0)
		repeatOnTimes = 1;
	else
		repeatOnTimes = 2;

	if (pass < repeatOnTimes) {
		if (millis() - lastBlinkMs > onMs) {
			isOn = !isOn;
			if (!isOn)
				pass++;
			digitalWrite(LED_OK, isOn);
			lastBlinkMs = millis();
		}
	}
	else if (millis() - lastBlinkMs > offMs) {
		pass = 0;
		lastBlinkMs = 0;
	}
}

void Robot::broadcastingStart(uint8_t measuringMode) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFreeBoardSlot; deviceNumber++)
		board[deviceNumber]->continuousReadingStart(0xFF, measuringMode);
}

void Robot::broadcastingStop() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFreeBoardSlot; deviceNumber++) {
		board[deviceNumber]->continuousReadingStop();
		delay(1); // TODO
	}
}

void Robot::canBusSniff() {
	while (!userBreak()) {
		blink();
		bool found = false;
		if (esp32CANBus->messageReceive()) {
			for (uint8_t deviceNumber = 0; deviceNumber < boardCount(); deviceNumber++)
				if (board[deviceNumber]->framePrint(esp32CANBus->rx_frame->MsgID, esp32CANBus->rx_frame->FIR.B.DLC,
					esp32CANBus->rx_frame->data.u8)) {
					found = true;
					break;
				}

			if (!found) {
				print("Not found Id: 0x%4X ", (String)esp32CANBus->rx_frame->MsgID);
				if (esp32CANBus->rx_frame->FIR.B.DLC > 0)
					print(", data: ");
				for (uint8_t i = 0; i < esp32CANBus->rx_frame->FIR.B.DLC; i++)
					print("0x2X ", esp32CANBus->rx_frame->data.u8[i]);
				print("\n\r");
			}
		}
	}
}

void Robot::canIdChange() {
	// Print all devices alive
	uint8_t last = 0;
	for (uint8_t boardNumber = 0; boardNumber < boardCount(); boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount(); deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0)
					print("%i.", ++last);
				else
					print(",");
				print(" %s", board[boardNumber]->name(deviceNumber));
				if (++currentCount == board[boardNumber]->devicesOnASingleBoard()) {
					currentCount = 0;
					print("\n\r");
				}
			}
	}
	if (last == 0)
		print("No boards\n\r");
	else {

		// Choose device to be changed
		print("Enter board [1 - %i]: ", last);
		uint32_t lastMs = millis();
		uint8_t selectedNumber = 0;
		bool any = false;
		while (millis() - lastMs < 30000 && !any || last > 9 && millis() - lastMs < 500 && any)
			if (Serial.available()) {
				selectedNumber = selectedNumber * 10 + (Serial.read() - 48);
				any = true;
				lastMs = millis();
			}

		if (selectedNumber > last) {
			if (any)
				print("invalid");
			else
				print("timeout\n\r");
		}
		else {

			// Find selected board
			last = 0;
			Board* selectedBoard = NULL;
			uint8_t selectedDeviceIndex = 0xFF;
			uint8_t maxInput = 0;
			for (uint8_t boardNumber = 0; boardNumber < boardCount() && selectedDeviceIndex == 0xFF; boardNumber++) {
				uint8_t currentCount = 0;
				for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount() && selectedDeviceIndex == 0xFF; deviceNumber++)
					if (board[boardNumber]->alive(deviceNumber)) {
						if (currentCount == 0) {
							if (++last == selectedNumber) {
								selectedBoard = board[boardNumber];
								selectedDeviceIndex = deviceNumber;
								maxInput = board[boardNumber]->deadOrAliveCount() / board[boardNumber]->devicesOnASingleBoard() - 1;
								break;
							}
						}

						if (++currentCount == board[boardNumber]->devicesOnASingleBoard())
							currentCount = 0;
					}
			}

			// Enter new id
			print("%i. %s\n\rEnter new board id [0..%i]: ", last, selectedBoard->name(), maxInput);
			lastMs = millis();
			uint8_t newDeviceNumber = 0;
			bool any = false;
			while ((millis() - lastMs < 30000 && !any || maxInput > 9 && millis() - lastMs < 500 && any) && newDeviceNumber < maxInput)
				if (Serial.available()) {
					newDeviceNumber = newDeviceNumber * 10 + (Serial.read() - 48);
					any = true;
					lastMs = millis();
				}

			if (newDeviceNumber > maxInput)
				print("timeout\n\r");
			else {

				// Change
				print("%i\n\rChange requested.\n\r", newDeviceNumber);
				selectedBoard->idChange(newDeviceNumber, selectedDeviceIndex);
				delay(500); // Delay for firmware handling of devices with the same ids.
			}
		}
	}
}

void Robot::commandProcess() {
	if (commandCurrent != NULL) {
		(*(commandCurrent->pointer))();
		if (commandCurrent != NULL)
			commandCurrent->firstProcess = false;
	}
}

void Robot::commandSet(struct Command* newCommand) {
	commandPrevious = commandCurrent;
	commandCurrent = newCommand;
	commandCurrent->firstProcess = true;
}

void Robot::commandUpdate(bool displayAlive, Command* displayAction, Command* switchAction) {
	static uint32_t lastUserActionMs = 0;
	static uint8_t uartRxCommandIndex = 0;
	static char uartRxCommandCumulative[10];
	const uint16_t TIMEOUT_MS = 2000;

	// If a button pressed, first execute its action
	if (displayAlive && displayAction != NULL)
		commandCurrent = displayAction;
	else if (switchAction != NULL)
		commandCurrent = switchAction;
	else { // Check keyboard
		if (Serial.available() || serial->available()) {
			lastUserActionMs = millis();
			uint8_t ch;
			if (Serial.available())
				ch = Serial.read();
			else
				ch = serial->read();

			if (ch != 13) //if received data different from ascii 13 (enter)
				uartRxCommandCumulative[uartRxCommandIndex++] = ch;	//add data to Rx_Buffer

			if (ch == 13 || uartRxCommandIndex >= 3 || ch == 'x') //if received data = 13
			{
				uartRxCommandCumulative[uartRxCommandIndex] = 0;
				uartRxCommandIndex = 0;

				print("Command: %s", uartRxCommandCumulative);

				uint8_t found = 0;
				for (uint8_t i = 0; i < COMMANDS_LIMIT; i++) {
					if (strcmp(commands[i]->shortcut, uartRxCommandCumulative) == 0) {
						print(" ok.\r\n");
						commandSet(commands[i]);
						//commandPrevious = commandCurrent;
						//commandCurrent = commands[i];
						//commandCurrent->firstProcess = true;
						found = 1;
						break;
					}
				}
				if (!found) {
					print(" not found.\r\n");
					uartRxCommandIndex = 0;
				}
			}
		}

		if (uartRxCommandIndex != 0 && millis() - lastUserActionMs > TIMEOUT_MS) {
			print(" Timeout.\r\n");
			uartRxCommandIndex = 0;
		}
	}
}

void Robot::devicesScan(bool verbose) {
	broadcastingStop();
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
		board[i]->devicesScan(verbose);
}

void Robot::errors() {
	static uint32_t lastDisplayMs = 0;
	if (strcmp(errorMessage, "") != 0) {
		if (millis() - lastDisplayMs > 10000 || lastDisplayMs == 0) {
			print("ERROR! %s\n\r", errorMessage);
			lastDisplayMs = millis();
			stopAll(); // Stop all motors
		}
	}
}

void Robot::firmwarePrint() {
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
		board[i]->firmwareRequest();
		uint32_t startMs = millis();
		while (millis() - startMs < 10)
			noLoopWithoutThis();
		board[i]->firmwareDisplay();
	}
}

void Robot::fps() {
	fpsMs[fpsNextIndex] = millis();
	if (++fpsNextIndex >= 3)
		fpsNextIndex = 0;
}

void Robot::fpsPrint() {
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
		board[i]->fpsRequest();
		uint32_t startMs = millis();
		while (millis() - startMs < 50)
			noLoopWithoutThis();
		board[i]->fpsDisplay();
	}
}

void Robot::menu() {
	// Print menu
	devicesScan(false);
	print("\r\n");
	bool any = false;
	uint8_t column = 1;
	uint8_t maxColumns = 2;
	for (uint8_t i = 0; i < COMMANDS_LIMIT && commands[i] != NULL; i++) {
		if ((commands[i]->menuLevel | menuLevel) == commands[i]->menuLevel) {
			print("%-3s - %-22s%s", commands[i]->shortcut, commands[i]->text, column == maxColumns ? "\n\r" : "");
			delay(2);
			any = true;
			if (column++ == maxColumns)
				column = 1;
		}
	}
	if (!any)
		print("Menu level %i empty.\r\n", menuLevel);
	else
		if (column != 1)
			print("\r\n");

	// Display errors
	for (uint8_t deviceNumber = 0; deviceNumber < nextFreeBoardSlot; deviceNumber++)
		if (board[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", board[deviceNumber]->errorCodeLast(), board[deviceNumber]->name(board[deviceNumber]->errorWasInDeviceNumber()));

	commandCurrent = &commandDoNothing;
}

void Robot::menuAdd(struct Command* command, char* shortcut, char* text, void (*pointer)(), uint8_t menuLevel) {
	if (nextFreeCommand >= COMMANDS_LIMIT) {
		strcpy(errorMessage, "COMMANDS_LIMIT exceeded.");
		return;
	}
	if (shortcut != 0)
		strcpy(command->shortcut, shortcut);
	if (text != 0)
		strcpy(command->text, text);
	command->pointer = pointer;
	command->menuLevel = menuLevel;
	commands[nextFreeCommand++] = command;
}

void Robot::messagesReceive() {
	while (esp32CANBus->messageReceive()) {
		uint32_t id = esp32CANBus->rx_frame->MsgID;
		bool any = false;
		for (uint8_t deviceGroupNumber = 0; deviceGroupNumber < nextFreeBoardSlot; deviceGroupNumber++) {
			if (board[deviceGroupNumber]->messageDecode(id, esp32CANBus->rx_frame->data.u8)) {
				any = true;
				break;
			}
		}

#define REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN false
#if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
		if (!any)
			print("Address device unknown: 0x%X\n\r", id);
#endif
	}
}

void Robot::motorTest() {
	print("Test motors\n\r");
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) 
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->aliveCount() > 0) 
			board[i]->test();
}

void Robot::noLoopWithoutThis() {
	blink(); // Keep-alive LED. Solder jumper must be shorted in order to work in mrm-esp32.
	messagesReceive();
	fps(); // Measure FPS. Less than 30 - a bad thing.
	verbosePrint(); // Print FPS and maybe some additional data
	errors();
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Robot::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

void Robot::stopAll() {
	broadcastingStop();
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->aliveCount() > 0)
			((MotorBoard*)board[i])->stop();
}

bool Robot::stressTest(bool firstPass) {
	const bool STOP_ON_ERROR = false;
	const uint16_t LOOP_COUNT = 10000;

	// Setup
	static uint32_t pass;
	static uint8_t lastPercent = 101;
	static uint8_t count[MRM_BOARD_COUNT];
	static uint32_t errors[MRM_BOARD_COUNT];
	if (firstPass) {
		print("Before test.\n\r");
		pass = 0;
		broadcastingStop();
		for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
			errors[i] = 0;
		for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
			delay(1);
			count[i] = board[i]->devicesScan(true);
		}
		print("Start.\n\r");
	}

	// Stress test
	uint8_t percent = 100 * pass / LOOP_COUNT;
	if (percent != lastPercent) {
		lastPercent = percent;
		print("%i %%\n\r", percent);
	}
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
		if (count[i] > 0) {
			delay(1);
			uint8_t cnt = board[i]->devicesScan(false);
			if (cnt != count[i]) {
				errors[i]++;
				print("***** %s: found %i, not %i.\n\r", board[i]->name(), cnt, count[i]);
				if (STOP_ON_ERROR)
					break;
			}
		}
	}

	// Results
	if (++pass >= LOOP_COUNT) {
		bool allOK = true;
		for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
			if (count[i] > 0 && errors[i] > 0) {
				print("%s: %i errors.\n\r", board[i]->name(), errors[i]);
				allOK = false;
			}
		if (allOK)
			print("No errors.");
		print("\n\r");
		return true;
	}
	else
		return false;
}

bool Robot::userBreak() {
	if (/*switchOn() ||*/ Serial.available() || serial->available()) {
		return true;
	}
	else
		return false;
}

void Robot::verbosePrint() {
	if (verbose) {
		static uint32_t lastMs = 0;
		if (lastMs == 0 || millis() - lastMs > 5000) {
			uint8_t lastFPS = (fpsNextIndex == 0 ? 2 : fpsNextIndex - 1);
			uint8_t firstFPS = fpsNextIndex;
			float fps;
			if (fpsMs[lastFPS] == 0 || fpsMs[lastFPS] - fpsMs[firstFPS] == 0)
				fps = 0;
			else
				fps = 2 * 1000 / (float)(fpsMs[lastFPS] - fpsMs[firstFPS]);
			print("%i fps\r\n", (int)round(fps));
			lastMs = millis();
		}
	}
}

void Robot::verboseToggle() {
	verbose = !verbose;
};

/** Print to all serial ports, pointer to list
*/
void Robot::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serialBT() != 0)
		serialBT()->print(buffer);
}
#include "mrm-col-can.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_col_can::Mrm_col_can(Robot* robot, uint8_t maxNumberOfBoards) : SensorBoard(robot, 1, "Color", maxNumberOfBoards, ID_MRM_COL_CAN) {
	readings = new std::vector<uint16_t[MRM_COL_CAN_COLORS]>(maxNumberOfBoards);
	_hsv = new std::vector<bool>(maxNumberOfBoards);
	_hue = new std::vector<uint8_t>(maxNumberOfBoards);
	_saturation = new std::vector<uint8_t>(maxNumberOfBoards);
	_value = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternByHSV = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternBy6Colors = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternRecognizedAtMs = new std::vector<uint32_t>(maxNumberOfBoards);
}

Mrm_col_can::~Mrm_col_can()
{
}

/** Add a mrm-col-can sensor
@param deviceName - device's name
*/
void Mrm_col_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_COL_CAN0_IN;
		canOut = CAN_ID_COL_CAN0_OUT;
		break;
	case 1:
		canIn = CAN_ID_COL_CAN1_IN;
		canOut = CAN_ID_COL_CAN1_OUT;
		break;
	case 2:
		canIn = CAN_ID_COL_CAN2_IN;
		canOut = CAN_ID_COL_CAN2_OUT;
		break;
	case 3:
		canIn = CAN_ID_COL_CAN3_IN;
		canOut = CAN_ID_COL_CAN3_OUT;
		break;
	case 4:
		canIn = CAN_ID_COL_CAN4_IN;
		canOut = CAN_ID_COL_CAN4_OUT;
		break;
	case 5:
		canIn = CAN_ID_COL_CAN5_IN;
		canOut = CAN_ID_COL_CAN5_OUT;
		break;
	case 6:
		canIn = CAN_ID_COL_CAN6_IN;
		canOut = CAN_ID_COL_CAN6_OUT;
		break;
	case 7:
		canIn = CAN_ID_COL_CAN7_IN;
		canOut = CAN_ID_COL_CAN7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-col-can");
		return;
	}

	//for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
	//	(*reading)[nextFree][i] = 0;

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Set gain
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param gainValue:
	0, 1x (default)
	1, 3.7x
	2, 16x
	3, 64x
*/
void Mrm_col_can::gain(uint8_t deviceNumber, uint8_t gainValue) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			gain(i, gainValue);
	else {
		canData[0] = CAN_COL_GAIN;
		canData[1] = gainValue;
		messageSend(canData, 2, deviceNumber);
	}
}

/** Set illumination intensity
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param current - 0 - 3
*/
void Mrm_col_can::illumination(uint8_t deviceNumber, uint8_t current) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			illumination(i, current);
	else {
		canData[0] = CAN_COL_ILLUMINATION_CURRENT;
		canData[1] = current;
		messageSend(canData, 2, deviceNumber);
	}
}

/** Set integration time
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param value - integration time will be value x 2.8 ms but double that in case of mode 2 (usual). value is between 0 and 255. value 18 is approx 10 FPS
*/
void Mrm_col_can::integrationTime(uint8_t deviceNumber, uint8_t value) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			integrationTime(i, value);
	else {
		canData[0] = CAN_COL_INTEGRATION_TIME;
		canData[1] = value;
		messageSend(canData, 2, deviceNumber);
	}
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_col_can::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				bool any = false;
				uint8_t startIndex = 0;
				switch (data[0]) {
				case CAN_COL_PATTERN_SENDING:
					print("Sensor %i, pattern %i: %i/%i/%i (H/S/V)\n\r", deviceNumber, data[1], data[2], data[3], data[4]);
					break;
				case COMMAND_SENSORS_MEASURE_SENDING:
					startIndex = 0;
					any = true;
					break;
				case CAN_COL_SENDING_COLORS_1_TO_3:
					(*readings)[deviceNumber][0] = (data[1] << 8) | data[2]; // blue
					(*readings)[deviceNumber][1] = (data[3] << 8) | data[4]; // green
					(*readings)[deviceNumber][2] = (data[5] << 8) | data[6]; // orange
					any = true;
					break;
				case CAN_COL_SENDING_COLORS_4_TO_6:
					(*readings)[deviceNumber][3] = (data[1] << 8) | data[2]; // red
					(*readings)[deviceNumber][4] = (data[3] << 8) | data[4]; // violet
					(*readings)[deviceNumber][5] = (data[5] << 8) | data[6]; // yellow
					(*_patternByHSV)[deviceNumber] = data[7] & 0xF;
					(*_patternBy6Colors)[deviceNumber] = data[7] >> 4;
					any = true;
					break;
				case CAN_COL_SENDING_HSV:
					(*_hue)[deviceNumber] = (data[1] << 8) | data[2]; 
					(*_saturation)[deviceNumber] = (data[3] << 8) | data[4];
					(*_value)[deviceNumber] = (data[5] << 8) | data[6];
					(*_patternByHSV)[deviceNumber] = data[7] & 0xF;
					(*_patternBy6Colors)[deviceNumber] = data[7] >> 4;
					(*_patternRecognizedAtMs)[deviceNumber] = millis();
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data, false);
					errorCode = 204;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}

/** Erase all patterns
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - in all sensors.
*/
void Mrm_col_can::patternErase(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			patternErase(i);
	else {
		canData[0] = CAN_COL_PATTERN_ERASE;
		messageSend(canData, 1, deviceNumber);
	}
}

/** Print HSV patterns
*/
void Mrm_col_can::patternPrint() {
	for (uint8_t deviceNumber = 0; deviceNumber < count(); deviceNumber++) {
		canData[0] = CAN_COL_PATTERN_REQUEST;
		messageSend(canData, 1, deviceNumber);
	}
}

/** Record a HSV pattern
@param patternNumber - 0 - MRM_COL_CAN_PATTERN_COUNT-1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_col_can::patternRecord(uint8_t patternNumber, uint8_t deviceNumber) {
	if (!_hsv || patternNumber >= MRM_COL_CAN_PATTERN_COUNT || deviceNumber >= nextFree) {
		strcpy(errorMessage, "Patt. err.");
		return;
	}
	canData[0] = CAN_COL_PATTERN_RECORD;
	canData[1] = patternNumber;
	messageSend(canData, 2, deviceNumber);
}

/** Record patterns manually
*/
void Mrm_col_can::patternsRecord() {
	// Select device
	uint8_t sensorsAlive = count();
	print("Enter sensor id [0..%i]: ", sensorsAlive - 1);
	uint16_t deviceNumber = robotContainer->serialReadNumber(8000, 500, nextFree - 1 <= 9, sensorsAlive - 1);
	if (deviceNumber == 0xFFFF) {
		print("Exit\n\r");
		return;
	}
	print("%i\n\r", deviceNumber);
	// Select pattern
	print("Enter pattern id [0..%i]: ", MRM_COL_CAN_PATTERN_COUNT - 1);
	uint16_t patternNumber = robotContainer->serialReadNumber(8000, 500, MRM_COL_CAN_PATTERN_COUNT - 1 <= 9, MRM_COL_CAN_PATTERN_COUNT - 1);
	if (patternNumber == 0xFFFF) {
		print("Exit\n\r");
		return;
	}
	print("%i\n\r", patternNumber);
	patternRecord(patternNumber, deviceNumber);
}

/** Analog readings
@param colorId - one of 6 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_col_can::reading(uint8_t colorId, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || colorId >= MRM_COL_CAN_COLORS) {
		strcpy(errorMessage, "mrm-col-can doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber][colorId];
}

/** Print all readings in a line
*/
void Mrm_col_can::readingsPrint() {
	print("Colors:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t colorId = 0; colorId < MRM_COL_CAN_COLORS; colorId++)
			print(" %3i", (*readings)[deviceNumber][colorId]);
	}
}


/** Instruction to sensor to switch to converting R, G, and B on board and return hue, saturation and value
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
*/
void Mrm_col_can::switchToHSV(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			switchToHSV(i);
	else {
		canData[0] = CAN_COL_SWITCH_TO_HSV;
		messageSend(canData, 1, deviceNumber);
		(*_hsv)[deviceNumber] = true;
	}
}


/** Instruction to sensor to start returning 6 raw colors
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
*/
void Mrm_col_can::switchTo6Colors(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			switchTo6Colors(i);
	else {
		canData[0] = CAN_COL_SWITCH_TO_6_COLORS;
		messageSend(canData, 1, deviceNumber);
		(*_hsv)[deviceNumber] = false;
	}
}


/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_col_can::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print(" | ");
				if ((*_hsv)[deviceNumber])
					print("H:%3i S:%3i V:%3i By HSV:%2i By col:%2i", (*_hue)[deviceNumber], (*_saturation)[deviceNumber], (*_value)[deviceNumber], (*_patternByHSV)[deviceNumber], (*_patternBy6Colors)[deviceNumber]);
				else
					print("Bl:%3i Gr:%3i Or:%3i Re:%3i Vi:%3i Ye:%3i", colorBlue(deviceNumber), colorGreen(deviceNumber), colorOrange(deviceNumber), colorRed(deviceNumber),
						colorViolet(deviceNumber), colorYellow(deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

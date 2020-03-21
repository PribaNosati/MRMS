#include "mrm-col-can.h"
#include <mrm-robot.h>

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_col_can::Mrm_col_can(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Color", maxNumberOfBoards) {
	readings = new std::vector<uint16_t[MRM_COL_CAN_COLORS]>(maxNumberOfBoards);
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
		canOut = CAN_ID_COL_CAN3_OUT;
		break;
	case 3:
		canIn = CAN_ID_COL_CAN3_IN;
		canOut = CAN_ID_COL_CAN4_OUT;
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
		strcpy(robotContainer->errorMessage, "Too many mrm-col-can");
		return;
	}

	//for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
	//	(*reading)[nextFree][i] = 0;

	SensorBoard::add(deviceName, canIn, canOut);
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
				case COMMAND_SENSORS_MEASURE_SENDING:
					startIndex = 0;
					any = true;
					break;
				case COMMAND_SENDING_COLORS_1_TO_3:
					(*readings)[deviceNumber][0] = (data[1] << 8) | data[2]; // blue
					(*readings)[deviceNumber][1] = (data[3] << 8) | data[4]; // green
					(*readings)[deviceNumber][2] = (data[5] << 8) | data[6]; // orange
					any = true;
					break;
				case COMMAND_SENDING_COLORS_4_TO_6:
					(*readings)[deviceNumber][3] = (data[1] << 8) | data[2]; // red
					(*readings)[deviceNumber][4] = (data[3] << 8) | data[4]; // violet
					(*readings)[deviceNumber][5] = (data[5] << 8) | data[6]; // yellow
					any = true;
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data);
					print("\n\r");
					errorCode = 204;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}

/** Analog readings
@param colorId - one of 6 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_col_can::reading(uint8_t colorId, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || colorId >= MRM_COL_CAN_COLORS) {
		strcpy(robotContainer->errorMessage, "mrm-col-can doesn't exist");
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
					print("| ");
				print("Bl:%3i Gr:%3i Or:%3i Re:%3i Vi:%3i Ye:%3i", colorBlue(deviceNumber), colorGreen(deviceNumber), colorOrange(deviceNumber), colorRed(deviceNumber),
					colorViolet(deviceNumber), colorYellow(deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

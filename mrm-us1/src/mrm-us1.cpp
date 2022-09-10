#include "mrm-us1.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_us1::Mrm_us1(Robot* robot, uint8_t maxNumberOfBoards) : SensorBoard(robot, 1, "US1", maxNumberOfBoards, ID_MRM_US1, 1) {
	readings = new std::vector<uint16_t>(maxNumberOfBoards);
}

Mrm_us1::~Mrm_us1()
{
}

/** Add a mrm-us1 sensor
@param deviceName - device's name
*/
void Mrm_us1::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_US1_0_IN;
		canOut = CAN_ID_US1_0_OUT;
		break;
	case 1:
		canIn = CAN_ID_US1_1_IN;
		canOut = CAN_ID_US1_1_OUT;
		break;
	case 2:
		canIn = CAN_ID_US1_2_IN;
		canOut = CAN_ID_US1_3_OUT;
		break;
	case 3:
		canIn = CAN_ID_US1_3_IN;
		canOut = CAN_ID_US1_4_OUT;
		break;
	case 4:
		canIn = CAN_ID_US1_4_IN;
		canOut = CAN_ID_US1_4_OUT;
		break;
	case 5:
		canIn = CAN_ID_US1_5_IN;
		canOut = CAN_ID_US1_5_OUT;
		break;
	case 6:
		canIn = CAN_ID_US1_6_IN;
		canOut = CAN_ID_US1_6_OUT;
		break;
	case 7:
		canIn = CAN_ID_US1_7_IN;
		canOut = CAN_ID_US1_7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-us1");
		return;
	}

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_us1::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
					case COMMAND_SENSORS_MEASURE_SENDING:
					{
						uint16_t mm = (data[2] << 8) | data[1];
						(*readings)[deviceNumber] = mm;
						(*_lastReadingMs)[deviceNumber] = millis();
					}
					break;
				// }
				default:
					robotContainer->print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 204;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}

/** Analog readings
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_us1::reading(uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		strcpy(errorMessage, "mrm-us1 doesn't exist");
		return 0;
	}
	alive(deviceNumber, true);
	if (started(deviceNumber))
		return (*readings)[deviceNumber];
	else
		return 0;
}

/** Print all readings in a line
*/
void Mrm_us1::readingsPrint() {
	robotContainer->print("US:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
			robotContainer->print(" %3i", (*readings)[deviceNumber]);
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_us1::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_US1_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//robotContainer->print("Start mrm-us1%i \n\r", deviceNumber); 
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//robotContainer->print("US confirmed\n\r");
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-us1 dead.\n\r");
		return false;
	}
	else
		return true;
}

/**Test
*/
void Mrm_us1::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					robotContainer->print("| ");
				robotContainer->print("%i ", reading(deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			robotContainer->print("\n\r");
	}
}

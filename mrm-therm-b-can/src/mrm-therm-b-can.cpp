#include "mrm-therm-b-can.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_therm_b_can::Mrm_therm_b_can(Robot* robot, uint8_t maxNumberOfBoards) : SensorBoard(robot, 1, "Thermo", maxNumberOfBoards, ID_MRM_THERM_B_CAN) {
	readings = new std::vector<int16_t>(maxNumberOfBoards);
}

Mrm_therm_b_can::~Mrm_therm_b_can()
{
}

/** Add a mrm-therm-b-can sensor
@param deviceName - device's name
*/
void Mrm_therm_b_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_THERM_B_CAN0_IN;
		canOut = CAN_ID_THERM_B_CAN0_OUT;
		break;
	case 1:
		canIn = CAN_ID_THERM_B_CAN1_IN;
		canOut = CAN_ID_THERM_B_CAN1_OUT;
		break;
	case 2:
		canIn = CAN_ID_THERM_B_CAN2_IN;
		canOut = CAN_ID_THERM_B_CAN2_OUT;
		break;
	case 3:
		canIn = CAN_ID_THERM_B_CAN3_IN;
		canOut = CAN_ID_THERM_B_CAN3_OUT;
		break;
	case 4:
		canIn = CAN_ID_THERM_B_CAN4_IN;
		canOut = CAN_ID_THERM_B_CAN4_OUT;
		break;
	case 5:
		canIn = CAN_ID_THERM_B_CAN5_IN;
		canOut = CAN_ID_THERM_B_CAN5_OUT;
		break;
	case 6:
		canIn = CAN_ID_THERM_B_CAN6_IN;
		canOut = CAN_ID_THERM_B_CAN6_OUT;
		break;
	case 7:
		canIn = CAN_ID_THERM_B_CAN7_IN;
		canOut = CAN_ID_THERM_B_CAN7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-therm-b-can\n\r");
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_therm_b_can::messageDecode(uint32_t canId, uint8_t data[8]){
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)){
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					int16_t temp = (data[2] << 8) | data[1];
					(*readings)[deviceNumber] = temp;
					(*_lastReadingMs)[deviceNumber] = millis();
				}
				break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data, false);
					errorCode = 205;
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
int16_t Mrm_therm_b_can::reading(uint8_t deviceNumber){
	if (deviceNumber >= nextFree) {
		strcpy(errorMessage, "Mrm_therm_b_can overflow.");
		return 0;
	}
	else
		if (started(deviceNumber))
			return (*readings)[deviceNumber];
		else
			return 0;
}

/** Print all readings in a line
*/
void Mrm_therm_b_can::readingsPrint() {
	print("Therm:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %i", reading(deviceNumber));
}


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_therm_b_can::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_THERM_B_CAN_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//print("Start mrm-therm-b-can-b2%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//print("Thermo confirmed\n\r"); 
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-therm-b-can dead.\n\r");
		return false;
	}
	else
		return true;
}

/**Test
*/
void Mrm_therm_b_can::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print(" ");
				print("%i ", reading(deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}

	//stop();
}


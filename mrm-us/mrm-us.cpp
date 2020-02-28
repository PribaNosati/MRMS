#include "mrm-us.h"

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_us::Mrm_us(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "US", maxNumberOfBoards) {
	readings = new std::vector<uint16_t[MRM_US_ECHOES_COUNT]>(maxNumberOfBoards);
	aliveTimeout = 100;
}

Mrm_us::~Mrm_us()
{
}

/** Add a mrm-us sensor
@param deviceName - device's name
*/
void Mrm_us::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_US0_IN;
		canOut = CAN_ID_US0_OUT;
		break;
	case 1:
		canIn = CAN_ID_US1_IN;
		canOut = CAN_ID_US1_OUT;
		break;
	case 2:
		canIn = CAN_ID_US2_IN;
		canOut = CAN_ID_US3_OUT;
		break;
	case 3:
		canIn = CAN_ID_US3_IN;
		canOut = CAN_ID_US4_OUT;
		break;
	case 4:
		canIn = CAN_ID_US4_IN;
		canOut = CAN_ID_US4_OUT;
		break;
	case 5:
		canIn = CAN_ID_US5_IN;
		canOut = CAN_ID_US5_OUT;
		break;
	case 6:
		canIn = CAN_ID_US6_IN;
		canOut = CAN_ID_US6_OUT;
		break;
	case 7:
		canIn = CAN_ID_US7_IN;
		canOut = CAN_ID_US7_OUT;
		break;
	default:
		strcpy(robotContainer->errorMessage, "Too many mrm-us");
		return;
	}

	//for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
	//	(*reading)[nextFree][i] = 0;

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_us::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (isForMe(canId, deviceNumber)) {
			messageDecodeCommon(deviceNumber);
			bool any = false;
			uint8_t startIndex = 0;
			switch (data[0]) {
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
				break;
				break;
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_SENSORS_MEASURE_SENDING:
				startIndex = 0;
				any = true;
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 204;
				errorInDeviceNumber = deviceNumber;
			}

			//if (any)
			//	for (uint8_t i = 0; i <= 2; i++)
			//		(*readings)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

			return true;
		}
	return false;
}

/** Analog readings
@param echoNumber - echoId
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_us::reading(uint8_t echoNumber, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || echoNumber > MRM_US_ECHOES_COUNT) {
		strcpy(robotContainer->errorMessage, "mrm-us doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber][echoNumber];
}

/** Print all readings in a line
*/
void Mrm_us::readingsPrint() {
	print("US:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t echoNumber = 0; echoNumber < MRM_US_ECHOES_COUNT; echoNumber++)
			print(" %3i", (*readings)[deviceNumber][echoNumber]);
	}
}


/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_us::test(BreakCondition breakWhen)
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print("| ");
				print("Echo:");
				for (uint8_t i = 0; i < MRM_US_ECHOES_COUNT; i++)
					print("%i ", (*readings)[deviceNumber][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

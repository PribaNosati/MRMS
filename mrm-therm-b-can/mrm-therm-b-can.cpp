#include "mrm-therm-b-can.h"
#include <mrm-robot.h>

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_therm_b_can::Mrm_therm_b_can(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Thermo", maxNumberOfBoards) {
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
		strcpy(robotContainer->errorMessage, "Too many mrm-therm-b-can\n\r");
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
			messageDecodeCommon(deviceNumber);
			switch (data[0]) {
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
				break;
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_SENSORS_MEASURE_SENDING: {
				int16_t temp = (data[2] << 8) | data[1];
				(*readings)[deviceNumber] = temp;
			}
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 205;
				errorInDeviceNumber = deviceNumber;
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
		strcpy(robotContainer->errorMessage, "Mrm_therm_b_can overflow.");
		return 0;
	}
	else
		return (*readings)[deviceNumber];
}

/** Print all readings in a line
*/
void Mrm_therm_b_can::readingsPrint() {
	print("Therm:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %i", (*readings)[deviceNumber]);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_therm_b_can::test(BreakCondition breakWhen)
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print(" ");
				print("%i ", (*readings)[deviceNumber]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}

	//continuousReadingStop();
}


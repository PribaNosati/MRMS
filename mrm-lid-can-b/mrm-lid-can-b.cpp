#include "mrm-lid-can-b.h"
#include <mrm-robot.h>

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_can_b::Mrm_lid_can_b(Robot* robot, uint8_t maxNumberOfBoards) :
	SensorBoard(robot, 1, "Lid2m", maxNumberOfBoards) {
	readings = new std::vector<uint16_t>(maxNumberOfBoards);
}

Mrm_lid_can_b::~Mrm_lid_can_b()
{
}

/** Add a mrm-lid-can-b device
@param deviceName - device's name
*/
void Mrm_lid_can_b::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_LID_CAN_B0_IN;
		canOut = CAN_ID_LID_CAN_B0_OUT;
		break;
	case 1:
		canIn = CAN_ID_LID_CAN_B1_IN;
		canOut = CAN_ID_LID_CAN_B1_OUT;
		break;
	case 2:
		canIn = CAN_ID_LID_CAN_B2_IN;
		canOut = CAN_ID_LID_CAN_B2_OUT;
		break;
	case 3:
		canIn = CAN_ID_LID_CAN_B3_IN;
		canOut = CAN_ID_LID_CAN_B3_OUT;
		break;
	case 4:
		canIn = CAN_ID_LID_CAN_B4_IN;
		canOut = CAN_ID_LID_CAN_B4_OUT;
		break;
	case 5:
		canIn = CAN_ID_LID_CAN_B5_IN;
		canOut = CAN_ID_LID_CAN_B5_OUT;
		break;
	case 6:
		canIn = CAN_ID_LID_CAN_B6_IN;
		canOut = CAN_ID_LID_CAN_B6_OUT;
		break;
	case 7:
		canIn = CAN_ID_LID_CAN_B7_IN;
		canOut = CAN_ID_LID_CAN_B7_OUT;
		break;
	case 8:
		canIn = CAN_ID_LID_CAN_B8_IN;
		canOut = CAN_ID_LID_CAN_B8_OUT;
		break;
	case 9:
		canIn = CAN_ID_LID_CAN_B9_IN;
		canOut = CAN_ID_LID_CAN_B9_OUT;
		break;
	case 10:
		canIn = CAN_ID_LID_CAN_B10_IN;
		canOut = CAN_ID_LID_CAN_B10_OUT;
		break;
	case 11:
		canIn = CAN_ID_LID_CAN_B11_IN;
		canOut = CAN_ID_LID_CAN_B11_OUT;
		break;
	case 12:
		canIn = CAN_ID_LID_CAN_B12_IN;
		canOut = CAN_ID_LID_CAN_B12_OUT;
		break;
	case 13:
		canIn = CAN_ID_LID_CAN_B13_IN;
		canOut = CAN_ID_LID_CAN_B13_OUT;
		break;
	case 14:
		canIn = CAN_ID_LID_CAN_B14_IN;
		canOut = CAN_ID_LID_CAN_B14_OUT;
		break;
	case 15:
		canIn = CAN_ID_LID_CAN_B15_IN;
		canOut = CAN_ID_LID_CAN_B15_OUT;
		break;
	default:
		strcpy(robotContainer->errorMessage, "Too many mrm-lid-can-b");
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Calibration, only once after production
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_can_b::calibration(uint8_t deviceNumber){
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibration(i);
	else{
		canData[0] = COMMAND_LID_CAN_B_CALIBRATE;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
	}
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_lid_can_b::messageDecode(uint32_t canId, uint8_t data[8]){
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (isForMe(canId, deviceNumber)) {
			messageDecodeCommon(deviceNumber);
			switch (data[0]) {
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			case COMMAND_SENSORS_MEASURE_SENDING: {
				uint16_t mm = (data[2] << 8) | data[1];
				(*readings)[deviceNumber] = mm;
			}
			break;
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 206;
				errorInDeviceNumber = deviceNumber;
			}
			return true;
		}
	}
	return false;
}

/** Analog readings
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_can_b::reading(uint8_t deviceNumber){
	if (deviceNumber > nextFree) {
		strcpy(robotContainer->errorMessage, "mrm-lid-can-b doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber];
}

/** Print all readings in a line
*/
void Mrm_lid_can_b::readingsPrint() {
	print("Lid2m:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %4i", (*readings)[deviceNumber]);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_lid_can_b::test(BreakCondition breakWhen)
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print(" ");
				print("%4i ", (*readings)[deviceNumber]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}

	//devicesScan(false);
	//print("%i devices alive.", aliveCount());
}
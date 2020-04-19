#include "mrm-lid-can-b2.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_can_b2::Mrm_lid_can_b2(Robot* robot, uint8_t maxNumberOfBoards) :
	SensorBoard(robot, 1, "Lid4m", maxNumberOfBoards) {
	readings = new std::vector<uint16_t>(maxNumberOfBoards);
}

Mrm_lid_can_b2::~Mrm_lid_can_b2()
{
}

/** Add a mrm-lid-can-b device
@param deviceName - device's name
*/
void Mrm_lid_can_b2::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_LID_CAN_B2_0_IN;
		canOut = CAN_ID_LID_CAN_B2_0_OUT;
		break;
	case 1:
		canIn = CAN_ID_LID_CAN_B2_1_IN;
		canOut = CAN_ID_LID_CAN_B2_1_OUT;
		break;
	case 2:
		canIn = CAN_ID_LID_CAN_B2_2_IN;
		canOut = CAN_ID_LID_CAN_B2_2_OUT;
		break;
	case 3:
		canIn = CAN_ID_LID_CAN_B2_3_IN;
		canOut = CAN_ID_LID_CAN_B2_3_OUT;
		break;
	case 4:
		canIn = CAN_ID_LID_CAN_B2_4_IN;
		canOut = CAN_ID_LID_CAN_B2_4_OUT;
		break;
	case 5:
		canIn = CAN_ID_LID_CAN_B2_5_IN;
		canOut = CAN_ID_LID_CAN_B2_5_OUT;
		break;
	case 6:
		canIn = CAN_ID_LID_CAN_B2_6_IN;
		canOut = CAN_ID_LID_CAN_B2_6_OUT;
		break;
	case 7:
		canIn = CAN_ID_LID_CAN_B2_7_IN;
		canOut = CAN_ID_LID_CAN_B2_7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-lid-can-b2");
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Calibration, only once after production
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_can_b2::calibration(uint8_t deviceNumber){
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibration(i);
	else{
		canData[0] = COMMAND_LID_CAN_B2_CALIBRATE;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 1, canData);
	}
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_lid_can_b2::messageDecode(uint32_t canId, uint8_t data[8]){
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)){
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint16_t mm = (data[2] << 8) | data[1];
					(*readings)[deviceNumber] = mm;
				}
				break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data);
					print("\n\r");
					errorCode = 202;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}

/** Ranging type
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param value - 1 short, 2 medium, 3 long.
*/
void Mrm_lid_can_b2::rangingType(uint8_t deviceNumber, uint8_t value) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibration(i);
	else {
		canData[0] = COMMAND_LID_CAN_B2_RANGING_TYPE;
		canData[1] = value;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
	}
}

/** Analog readings
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_can_b2::reading(uint8_t deviceNumber){
	if (deviceNumber >= nextFree) {
		strcpy(errorMessage, "mrm-lid-can-b2 doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber];
}

/** Print all readings in a line
*/
void Mrm_lid_can_b2::readingsPrint() {
	print("Lid4m:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %4i", (*readings)[deviceNumber]);
}

/**Test
*/
void Mrm_lid_can_b2::test()
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
}


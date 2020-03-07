#include "mrm-ir-finder-can.h"
#include <mrm-robot.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_ir_finder_can::Mrm_ir_finder_can(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "IRFindCan", maxNumberOfBoards) {
	readings = new std::vector<uint16_t[MRM_IR_FINDER_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
}

Mrm_ir_finder_can::~Mrm_ir_finder_can()
{
}

/** Add a mrm-ir-finder sensor
@param deviceName - device's name
*/
void Mrm_ir_finder_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_IR_FINDER_CAN0_IN;
		canOut = CAN_ID_IR_FINDER_CAN0_OUT;
		break;
	case 1:
		canIn = CAN_ID_IR_FINDER_CAN1_IN;
		canOut = CAN_ID_IR_FINDER_CAN1_OUT;
		break;
	case 2:
		canIn = CAN_ID_IR_FINDER_CAN2_IN;
		canOut = CAN_ID_IR_FINDER_CAN2_OUT;
		break;
	case 3:
		canIn = CAN_ID_IR_FINDER_CAN3_IN;
		canOut = CAN_ID_IR_FINDER_CAN3_OUT;
		break;
	case 4:
		canIn = CAN_ID_IR_FINDER_CAN4_IN;
		canOut = CAN_ID_IR_FINDER_CAN4_OUT;
		break;
	case 5:
		canIn = CAN_ID_IR_FINDER_CAN5_IN;
		canOut = CAN_ID_IR_FINDER_CAN5_OUT;
		break;
	case 6:
		canIn = CAN_ID_IR_FINDER_CAN6_IN;
		canOut = CAN_ID_IR_FINDER_CAN6_OUT;
		break;
	case 7:
		canIn = CAN_ID_IR_FINDER_CAN7_IN;
		canOut = CAN_ID_IR_FINDER_CAN7_OUT;
		break;
	default:
		strcpy(robotContainer->errorMessage, "Too many mrm-ir-finder-can");
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_ir_finder_can::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				bool any = false;
				uint8_t startIndex = 0;
				switch (data[0]) {
				case COMMAND_IR_FINDER_CAN_SENDING_SENSORS_1_TO_3:
					startIndex = 0;
					any = true;
					break;
				case COMMAND_IR_FINDER_CAN_SENDING_SENSORS_4_TO_6:
					startIndex = 3;
					any = true;
					break;
				case COMMAND_IR_FINDER_CAN_SENDING_SENSORS_7_TO_9:
					startIndex = 6;
					any = true;
					break;
				case COMMAND_IR_FINDER_CAN_SENDING_SENSORS_10_TO_12:
					startIndex = 9;
					any = true;
					break;
				case COMMAND_SENSORS_MEASURE_CALCULATED_SENDING:
					angle = (data[1] << 8 | data[2]) - 180;
					distance = data[3] << 8 | data[4];
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data);
					print("\n\r");
					errorCode = 201;
					errorInDeviceNumber = deviceNumber;
				}

				if (any)
					for (uint8_t i = 0; i <= 2; i++)
						(*readings)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];
			}
			return true;
		}
	return false;
}

/** Analog readings
@param receiverNumberInSensor - single IR receiver in mrm-ir-finder
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ir_finder_can::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_IR_FINDER_CAN_SENSOR_COUNT) {
		strcpy(robotContainer->errorMessage, "mrm-ir-finder-can doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_ir_finder_can::readingsPrint() {
	print("IRBall:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber)) {
			for (uint8_t irNo = 0; irNo < MRM_IR_FINDER_CAN_SENSOR_COUNT; irNo++)
				print(" %3i", (*readings)[deviceNumber][irNo]);
		}
}

/**Test
*/
void Mrm_ir_finder_can::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print("| ");
				for (uint8_t i = 0; i < MRM_IR_FINDER_CAN_SENSOR_COUNT; i++)
					print("%i ", (*readings)[deviceNumber][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

/**Test
*/
void Mrm_ir_finder_can::testCalculated()
{
	static uint32_t lastMs = 0;
	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) 
				print("%i deg., dist: %i\n\r", angle, distance);
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}


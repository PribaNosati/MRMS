#include "mrm-lid-can-b2.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_can_b2::Mrm_lid_can_b2(Robot* robot, uint8_t maxNumberOfBoards) : SensorBoard(robot, 1, "Lid4m", maxNumberOfBoards, ID_MRM_LID_CAN_B2) {
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
		messageSend(canData, 1, deviceNumber);
	}
}


/** Reset sensor's non-volatile memory to defaults (distance mode, timing budget, region of interest, and measurement time, but leaves CAN Bus id intact
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - resets all.
*/
void Mrm_lid_can_b2::defaults(uint8_t deviceNumber) {
	timingBudget(deviceNumber);
	robotContainer->delayMs(50); // Allow 50 ms for flash to be written
	measurementTime(deviceNumber);
	robotContainer->delayMs(50);
	distanceMode(deviceNumber);
	robotContainer->delayMs(50);
	roi(deviceNumber);
}


/** Distance mode. Short mode has better ambient light immunity but the maximum distance is limited to 1.3 m. Long distance ranges up to
	4 m but is less performant under ambient light. Stored in sensors non-volatile memory. Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param isShort - if not short, long.
*/
void Mrm_lid_can_b2::distanceMode(uint8_t deviceNumber, bool isShort) {
	if (deviceNumber == 0xFF) {
		for (uint8_t i = 0; i < nextFree; i++)
			if (alive(i)) {
				distanceMode(i, isShort);
				delay(1);
			}
	}
	else {
		canData[0] = COMMAND_LID_CAN_B2_DISTANCE_MODE;
		canData[1] = isShort;
		messageSend(canData, 2, deviceNumber);
	}
}


/** Measurement time (IMP) in ms. IMP must be >= TB. Probably the best value is IMP = TB. Stored in sensors non-volatile memory.
Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param ms - default 100.
*/
void Mrm_lid_can_b2::measurementTime(uint8_t deviceNumber, uint16_t ms) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			measurementTime(i, ms);
	else if (alive(deviceNumber)) {
		canData[0] = COMMAND_LID_CAN_B2_MEASUREMENT_TIME;
		canData[1] = ms & 0xFF;
		canData[2] = ms >> 8;
		messageSend(canData, 3, deviceNumber);
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
					(*_lastReadingMs)[deviceNumber] = millis();
				}
				break;
				case COMMAND_INFO_SENDING_1:
					print("%s: %s dist., budget %i ms, %ix%i, intermeas. %i ms\n\r", name(deviceNumber), data[1] ? "short" : "long", data[2] | (data[3] << 8),
						data[4] & 0xFF, data[5] & 0xFF, data[6] | (data[7] << 8));
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data, false);
					errorCode = 202;
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
uint16_t Mrm_lid_can_b2::reading(uint8_t deviceNumber){
	if (deviceNumber >= nextFree) {
		strcpy(errorMessage, "mrm-lid-can-b2 doesn't exist");
		return 0;
	}
	if (started(deviceNumber))
		return (*readings)[deviceNumber] == 0 ? 2000 : (*readings)[deviceNumber];
	else
		return 0;
}

/** Print all readings in a line
*/
void Mrm_lid_can_b2::readingsPrint() {
	print("Lid4m:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			print(" %4i", reading(deviceNumber));
}


/** ROI, region of interest, a matrix from 4x4 up to 16x16 (x, y). Smaller x and y - smaller view angle. Stored in sensors non-volatile memory.
Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param x - default 16.
@param y - default 16.
*/
void Mrm_lid_can_b2::roi(uint8_t deviceNumber, uint8_t x, uint8_t y) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			roi(i, x, y);
	else if (alive(deviceNumber)) {
		delay(1);
		canData[0] = COMMAND_LID_CAN_B2_ROI;
		canData[1] = x;
		canData[2] = y;
		messageSend(canData, 3, deviceNumber);
	}
}


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_lid_can_b2::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_LID_CAN_B2_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//print("Start mrm-lid-can-b2%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//print("Lidar confirmed\n\r"); 
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-lid-can-b2 dead.\n\r");
		return false;
	}
	else
		return true;
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
				print("%i ", reading(deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}


/** Timing budget (TB) in ms. TB improves the measurement reliability but increases power consumption. Stored in sensors non-volatile memory.
	Set before measurement time as measurement time checks this value and returns error if not appropriate. Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param ms - (TB) in ms possible values [20, 50, 100, 200, 500]. Default 100.
*/
void Mrm_lid_can_b2::timingBudget(uint8_t deviceNumber, uint16_t ms) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			timingBudget(i, ms);
	else if (alive(deviceNumber)) {
		delay(1);
		canData[0] = COMMAND_LID_CAN_B2_TIMING_BUDGET;
		canData[1] = ms & 0xFF;
		canData[2] = ms >> 8;
		messageSend(canData, 3, deviceNumber);
	}
}


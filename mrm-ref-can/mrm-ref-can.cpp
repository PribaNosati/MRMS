#include "mrm-ref-can.h"

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_ref_can::Mrm_ref_can(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "ReflArray", maxNumberOfBoards) {
	readings = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	calibrationData = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	dataFresh = new std::vector<uint8_t>(maxNumberOfBoards);
	measuringModeLimit = 2;
	centerOfMeasurements = new std::vector<uint16_t>(maxNumberOfBoards);
}

Mrm_ref_can::~Mrm_ref_can()
{
}

/** Add a mrm-ref-can sensor
@param deviceName - device's name
*/
void Mrm_ref_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_REF_CAN0_IN;
		canOut = CAN_ID_REF_CAN0_OUT;
		break;
	case 1:
		canIn = CAN_ID_REF_CAN1_IN;
		canOut = CAN_ID_REF_CAN1_OUT;
		break;
	case 2:
		canIn = CAN_ID_REF_CAN2_IN;
		canOut = CAN_ID_REF_CAN2_OUT;
		break;
	case 3:
		canIn = CAN_ID_REF_CAN3_IN;
		canOut = CAN_ID_REF_CAN3_OUT;
		break;
	case 4:
		canIn = CAN_ID_REF_CAN4_IN;
		canOut = CAN_ID_REF_CAN4_OUT;
		break;
	case 5:
		canIn = CAN_ID_REF_CAN5_IN;
		canOut = CAN_ID_REF_CAN5_OUT;
		break;
	case 6:
		canIn = CAN_ID_REF_CAN6_IN;
		canOut = CAN_ID_REF_CAN6_OUT;
		break;
	case 7:
		canIn = CAN_ID_REF_CAN7_IN;
		canOut = CAN_ID_REF_CAN7_OUT;
		break;
	default:
		strcpy(robotContainer->errorMessage, "Too many mrm-ref-cans");
		return;
	}
	(*dataFresh)[nextFree] = 0xFF;
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Calibrate the array
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
*/
void Mrm_ref_can::calibrate(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibrate(i);
	else {
		canData[0] = COMMAND_REF_CAN_CALIBRATE;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
	}
}

/** Get local calibration data
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::calibrationDataGet(uint8_t receiverNumberInSensor, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT) {
		strcpy(robotContainer->errorMessage, "mrm-ref-can doesn't exist");
		return 0;
	}
	return (*calibrationData)[deviceNumber][receiverNumberInSensor];
}

/** Request sensor to send calibration data
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param waitForResult - Blocks program flow till results return.
*/
void Mrm_ref_can::calibrationDataRequest(uint8_t deviceNumber, bool waitForResult) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibrate(i);
	else {
		if (waitForResult)
			dataFreshCalibrationSet(deviceNumber, false);
		canData[0] = COMMAND_REF_CAN_CALIBRATION_DATA_REQUEST;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
		if (waitForResult)
			while (!dataCalibrationFreshAsk(deviceNumber))
				robotContainer->messagesReceive();
	}
}

/** Print all calibration in a line
*/
void Mrm_ref_can::calibrationPrint() {
	print("Cal:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
			if (alive(deviceNumber))
				print(" %3i", calibrationDataGet(irNo, deviceNumber));
	}
}

/** Dark?
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - yes or no.
*/
bool Mrm_ref_can::dark(uint8_t receiverNumberInSensor, uint8_t deviceNumber) { 
	if (measuringMode == 0) // Analog readings
		return (*readings)[deviceNumber][receiverNumberInSensor] < (*calibrationData)[deviceNumber][receiverNumberInSensor];
	else // Digital readings
		return (*readings)[deviceNumber][receiverNumberInSensor];
}


/** Set calibration data freshness
@param areFresh - set value
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
*/
void Mrm_ref_can::dataFreshCalibrationSet(bool areFresh, uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			dataFreshCalibrationSet(i);
	else
		(*dataFresh)[deviceNumber] &= areFresh ? 0b11111111 : 0b11100011;
}

/** Set readings data freshness
@param areFresh - set value
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
*/
void Mrm_ref_can::dataFreshReadingsSet(bool areFresh, uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			dataFreshReadingsSet(i);
	else
		(*dataFresh)[deviceNumber] &= areFresh ? 0b11111111 : 0b00011111;
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_ref_can::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			messageDecodeCommon(deviceNumber);
			bool anyReading = false;
			bool anyCalibrationData = false;
			uint8_t startIndex = 0;
			switch (data[0]) {
			case COMMAND_REF_CAN_CALIBRATION_DATA_1_TO_3:
				startIndex = 0;
				anyCalibrationData = true;
				(*dataFresh)[deviceNumber] |= 0b00010000;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_4_TO_6:
				startIndex = 3;
				anyCalibrationData = true;
				(*dataFresh)[deviceNumber] |= 0b00001000;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_7_TO_9:
				startIndex = 6;
				anyCalibrationData = true;
				(*dataFresh)[deviceNumber] |= 0b00000100;
				break;
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
				break;
			case COMMAND_FIRMWARE_SENDING:
				(*firmwareVersionLast)[deviceNumber] = (data[2] << 8) | data[1];
				break;
			case COMMAND_FPS_SENDING:
				fpsLast = (data[2] << 8) | data[1];
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3:
				startIndex = 0;
				anyReading = true;
				(*dataFresh)[deviceNumber] |= 0b10000000;
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6:
				startIndex = 3;
				anyReading = true;
				(*dataFresh)[deviceNumber] |= 0b01000000;
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9:
				startIndex = 6;
				anyReading = true;
				(*dataFresh)[deviceNumber] |= 0b00100000;
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_CENTER:
				(*centerOfMeasurements)[deviceNumber] = (uint16_t)((data[2] << 8) | data[1]);

				(*readings)[deviceNumber][0] = (data[3] & 0b10000000) >> 7;
				(*readings)[deviceNumber][1] = (data[3] & 0b01000000) >> 6;
				(*readings)[deviceNumber][2] = (data[3] & 0b00100000) >> 5;
				(*readings)[deviceNumber][3] = (data[3] & 0b00010000) >> 4;
				(*readings)[deviceNumber][4] = (data[3] & 0b00001000) >> 3;
				(*readings)[deviceNumber][5] = (data[3] & 0b00000100) >> 2;
				(*readings)[deviceNumber][6] = (data[3] & 0b00000010) >> 1;
				(*readings)[deviceNumber][7] = data[3] & 0b00000001;
				(*readings)[deviceNumber][8] = data[4];

				(*dataFresh)[deviceNumber] |= 0b11100000;
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 201;
				errorInDeviceNumber = deviceNumber;
			}

			if (anyReading)
				for (uint8_t i = 0; i <= 2; i++)
					(*readings)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

			if (anyCalibrationData)
				for (uint8_t i = 0; i <= 2; i++)
					(*calibrationData)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

			return true;
		}
	return false;
}

/** Readings, can be analog or digital, depending on measuring mode
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT) {
		strcpy(robotContainer->errorMessage, "mrm-ref-can doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_ref_can::readingsPrint() {
	print("Refl:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
			if (alive(deviceNumber))
				print(" %3i", (*readings)[deviceNumber][irNo]);
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_ref_can::test(BreakCondition breakWhen)
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print("| ");
				for (uint8_t i = 0; i < MRM_REF_CAN_SENSOR_COUNT; i++)
					print("%i ", (*readings)[deviceNumber][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}

	//continuousReadingStop();
}


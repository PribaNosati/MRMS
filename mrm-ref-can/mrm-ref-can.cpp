#include "mrm-ref-can.h"
#include <mrm-robot.h>

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_ref_can::Mrm_ref_can(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "ReflArray", maxNumberOfBoards) {
	_reading = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	calibrationDataBright = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	calibrationDataDark = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
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

/** Any dark or bright
@param dark - any dark? Otherwise, any bright?
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
bool Mrm_ref_can::any(bool dark, uint8_t deviceNumber) {
	if (readingDigitalAndCenter)
		for (uint8_t i = 0; i < 8; i++)
			if ((*_reading)[deviceNumber][i] == 1)
				return true;
}

/** Calibrate the array
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
*/
void Mrm_ref_can::calibrate(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibrate(i);
	else if (alive(deviceNumber)){
		canData[0] = COMMAND_REF_CAN_CALIBRATE;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
	}
	robotContainer->actionEnd();
}

/** Get local calibration data
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param isDark - if true calibration for dark, otherwise for bright
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::calibrationDataGet(uint8_t receiverNumberInSensor, bool isDark, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT) {
		strcpy(robotContainer->errorMessage, "mrm-ref-can doesn't exist");
		return 0;
	}
	alive(deviceNumber);
	return (isDark ? (*calibrationDataDark) : (*calibrationDataBright))[deviceNumber][receiverNumberInSensor];
}

/** Request sensor to send calibration data
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param waitForResult - Blocks program flow till results return.
*/
void Mrm_ref_can::calibrationDataRequest(uint8_t deviceNumber, bool waitForResult) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibrationDataRequest(i, waitForResult);
	else if (alive(deviceNumber)){
		if (waitForResult)
			dataFreshCalibrationSet(false, deviceNumber);
		canData[0] = COMMAND_REF_CAN_CALIBRATION_DATA_REQUEST;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
		if (waitForResult) {
			uint32_t ms = millis();
			while (!dataCalibrationFreshAsk(deviceNumber)) {
				robotContainer->noLoopWithoutThis();
				if (millis() - ms > 1000) {
					strcpy(robotContainer->errorMessage, "Cal. data timeout.");
					break;
				}
			}
		}
	}
}

/** Print all calibration in a line
*/
void Mrm_ref_can::calibrationPrint() {
	print("Calibration.\n\r");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (alive(deviceNumber)) {
			print("Dark: ");
			for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
				print(" %3i", calibrationDataGet(irNo, true, deviceNumber));
			print("\n\rBright: ");
			for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
				print(" %3i", calibrationDataGet(irNo, false, deviceNumber));
			print("\n\r");
		}
}

/** Dark?
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - yes or no.
*/
bool Mrm_ref_can::dark(uint8_t receiverNumberInSensor, uint8_t deviceNumber) {
	alive(deviceNumber, true);
	if (measuringMode == 0) // Analog readings
		return (*_reading)[deviceNumber][receiverNumberInSensor] < ((*calibrationDataDark)[deviceNumber][receiverNumberInSensor] + (*calibrationDataBright)[deviceNumber][receiverNumberInSensor]) / 2;
	else // Digital readings
		return (*_reading)[deviceNumber][receiverNumberInSensor];
}


/** Set calibration data freshness
@param setToFresh - set value to be fresh. Otherwise set to not to be.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
*/
void Mrm_ref_can::dataFreshCalibrationSet(bool setToFresh, uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			dataFreshCalibrationSet(setToFresh, i);
	else
		if (setToFresh)
			(*dataFresh)[deviceNumber] |= 0b00011100;
		else
			(*dataFresh)[deviceNumber] &= 0b11100011;
}

/** Set readings data freshness
@param setToFresh - set value
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
*/
void Mrm_ref_can::dataFreshReadingsSet(bool setToFresh, uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			dataFreshReadingsSet(setToFresh, i);
	else
		if (setToFresh)
			(*dataFresh)[deviceNumber] |= 0b11100000;
		else
			(*dataFresh)[deviceNumber] &= 0b00011111;
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
			bool anyCalibrationDataDark = false;
			bool anyCalibrationDataBright = false;
			uint8_t startIndex = 0;
			switch (data[0]) {
			case COMMAND_REF_CAN_CALIBRATION_DATA_DARK_1_TO_3:
				// todo - dataFresh only 8 bits so the first 3 messages do not work
				startIndex = 0;
				anyCalibrationDataDark = true;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_DARK_4_TO_6:
				startIndex = 3;
				anyCalibrationDataDark = true;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_DARK_7_TO_9:
				startIndex = 6;
				anyCalibrationDataDark = true;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_1_TO_3:
				startIndex = 0;
				anyCalibrationDataBright = true;
				(*dataFresh)[deviceNumber] |= 0b00010000;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_4_TO_6:
				startIndex = 3;
				anyCalibrationDataBright = true;
				(*dataFresh)[deviceNumber] |= 0b00001000;
				break;
			case COMMAND_REF_CAN_CALIBRATION_DATA_BRIGHT_7_TO_9:
				startIndex = 6;
				anyCalibrationDataBright = true;
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

				(*_reading)[deviceNumber][0] = (data[3] & 0b10000000) >> 7;
				(*_reading)[deviceNumber][1] = (data[3] & 0b01000000) >> 6;
				(*_reading)[deviceNumber][2] = (data[3] & 0b00100000) >> 5;
				(*_reading)[deviceNumber][3] = (data[3] & 0b00010000) >> 4;
				(*_reading)[deviceNumber][4] = (data[3] & 0b00001000) >> 3;
				(*_reading)[deviceNumber][5] = (data[3] & 0b00000100) >> 2;
				(*_reading)[deviceNumber][6] = (data[3] & 0b00000010) >> 1;
				(*_reading)[deviceNumber][7] = data[3] & 0b00000001;
				(*_reading)[deviceNumber][8] = data[4];

				(*dataFresh)[deviceNumber] |= 0b11100000;
				break;
			default:
				print("Unknown command. ");
				messagePrint(canId, 8, data);
				print("\n\r");
				errorCode = 201;
				errorInDeviceNumber = deviceNumber;
			}

			if (anyReading)
				for (uint8_t i = 0; i <= 2; i++)
					(*_reading)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

			if (anyCalibrationDataBright)
				for (uint8_t i = 0; i <= 2; i++)
					(*calibrationDataBright)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

			if (anyCalibrationDataDark)
				for (uint8_t i = 0; i <= 2; i++)
					(*calibrationDataDark)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

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
	alive(deviceNumber, true);
	return (*_reading)[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_ref_can::readingsPrint() {
	print("Refl:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
			if (alive(deviceNumber))
				print(" %3i", (*_reading)[deviceNumber][irNo]);
	}
}

/** Reset
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
*/
void Mrm_ref_can::reset(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			reset(i);
	else {
		canData[0] = COMMAND_RESET;
		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 1, canData);
	}
	robotContainer->actionEnd();
}

/**Test
*/
void Mrm_ref_can::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print("| ");
				for (uint8_t i = 0; i < MRM_REF_CAN_SENSOR_COUNT; i++)
					print("%i ", (*_reading)[deviceNumber][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}


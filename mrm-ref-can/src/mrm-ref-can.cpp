#include "mrm-ref-can.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_ref_can::Mrm_ref_can(Robot* robot, uint8_t maxNumberOfBoards) : SensorBoard(robot, 1, "ReflArray", maxNumberOfBoards, ID_MRM_REF_CAN) {
	_reading = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	calibrationDataBright = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	calibrationDataDark = new std::vector<uint16_t[MRM_REF_CAN_SENSOR_COUNT]>(maxNumberOfBoards);
	dataFresh = new std::vector<uint8_t>(maxNumberOfBoards);
	_mode = new std::vector<uint8_t>(maxNumberOfBoards);
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
		strcpy(errorMessage, "Too many mrm-ref-cans");
		return;
	}
	(*dataFresh)[nextFree] = 0xFF;
	SensorBoard::add(deviceName, canIn, canOut);
}

/** If analog mode not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_ref_can::analogStarted(uint8_t deviceNumber) {
	if ((*_mode)[deviceNumber] != ANALOG_VALUES || millis() - (*_lastReadingMs)[deviceNumber] > MRM_REF_CAN_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//print("Start analog \n\r"); 
		(*_lastReadingMs)[deviceNumber] = 0;
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0); // As analog
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//print("Analog confirmed\n\r"); 
					(*_mode)[deviceNumber] = ANALOG_VALUES;
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-ref-can dead.\n\r");
		return false;
	}
	else
		return true;
}

/** Any dark or bright
@param dark - any dark? Otherwise, any bright?
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param firstTransistor - start checking from this transistor
@param lastTransistor - do not check after this one
*/
bool Mrm_ref_can::any(bool dark, uint8_t deviceNumber, uint8_t fistTransistor, uint8_t lastTransistor) {
	// If DIGITAL_AND_BRIGHT_CENTER started, bright will be 1. If DIGITAL_AND_DARK_CENTER started, dark will be 1. Therefore, complication:
	if (!digitalStarted(deviceNumber, false, false) && !digitalStarted(deviceNumber, true, false))
		if (!digitalStarted(deviceNumber, dark))
			return false;

	for (uint8_t i = fistTransistor; i < min(lastTransistor, (uint8_t)8); i++)
		if ((*_mode)[deviceNumber] == DIGITAL_AND_DARK_CENTER) {
			if ((*_reading)[deviceNumber][i] == dark ? 1 : 0)
				return true;
		}
		else
			if ((*_reading)[deviceNumber][i] == dark ? 0 : 1)
				return true;
	return false;
}

/** Calibrate the array
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
*/
void Mrm_ref_can::calibrate(uint8_t deviceNumber) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++) {
			if (i != 0)
				robotContainer->delayMicros(800);
			calibrate(i);
		}
	else if (alive(deviceNumber)){
		aliveSet(false, deviceNumber);
		print("Calibrating %s...", name(deviceNumber));
		canData[0] = COMMAND_REF_CAN_CALIBRATE;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 1, canData);
		uint32_t startMs = millis();
		bool ok = false;
		while (millis() - startMs < 10000) {
			robotContainer->noLoopWithoutThis();
			if (alive(deviceNumber)) {
				ok = true;
				break;
			}
		}
		if (ok)
			print("OK\n\r");
		else
			print("timeout\n\r");
	}
	robotContainer->end();
}

/** Get local calibration data
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param isDark - if true calibration for dark, otherwise for bright
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::calibrationDataGet(uint8_t receiverNumberInSensor, bool isDark, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT) {
		strcpy(errorMessage, "mrm-ref-can doesn't exist");
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
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 1, canData);
		if (waitForResult) {
			uint32_t ms = millis();
			while (!dataCalibrationFreshAsk(deviceNumber)) {
				robotContainer->noLoopWithoutThis();
				if (millis() - ms > 1000) {
					strcpy(errorMessage, "Cal. data timeout.");
					break;
				}
			}
		}
	}
}

/** Print all calibration in a line
*/
void Mrm_ref_can::calibrationPrint() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (alive(deviceNumber)) {
			print("Calibration for %s.\n\r", name());
			print("Dark: ");
			for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
				print(" %3i", calibrationDataGet(irNo, true, deviceNumber));
			print("\n\rBright: ");
			for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
				print(" %3i", calibrationDataGet(irNo, false, deviceNumber));
			print("\n\r");
		}
}

/** Center of measurements, like center of the line
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - calibrate all sensors.
@param ofDark - center of dark. Otherwise center of bright.
@return - 1000 - 9000. 1000 means center exactly under first phototransistor (denoted with "1" on the printed circuit board), 5000 is center transistor.
*/
uint16_t Mrm_ref_can::center(uint8_t deviceNumber, bool ofDark) { 
	if (digitalStarted(deviceNumber, ofDark))
		return (*centerOfMeasurements)[deviceNumber];
	else
		return false;
}

/** Dark?
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param fromAnalog - from analog local values. If not, sensor-supplied center data.
@return - yes or no.
*/
bool Mrm_ref_can::dark(uint8_t receiverNumberInSensor, uint8_t deviceNumber, bool fromAnalog) {
	alive(deviceNumber, true);
	if (fromAnalog) {// Analog readings
		if (analogStarted(deviceNumber))
			return (*_reading)[deviceNumber][receiverNumberInSensor] < ((*calibrationDataDark)[deviceNumber][receiverNumberInSensor] + (*calibrationDataBright)[deviceNumber][receiverNumberInSensor]) / 2;
		else
			return false;
	}
	else { // Digital readings
		if (!digitalStarted(deviceNumber, false, false) && !digitalStarted(deviceNumber, true, false))
			if (!digitalStarted(deviceNumber, true))
				return false;
		if ((*_mode)[deviceNumber] == DIGITAL_AND_DARK_CENTER) 
			return (*_reading)[deviceNumber][receiverNumberInSensor] == 1;
		else
			return (*_reading)[deviceNumber][receiverNumberInSensor] == 0;
	}
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

/** If digital mode with dark center not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param darkCenter - Center of dark. If not, center of bright.
@param startIfNot - If not already started, start now.
@return - started or not
*/
bool Mrm_ref_can::digitalStarted(uint8_t deviceNumber, bool darkCenter, bool startIfNot) {
	if ((*_mode)[deviceNumber] != DIGITAL_AND_DARK_CENTER || millis() - (*_lastReadingMs)[deviceNumber] > MRM_REF_CAN_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		if (startIfNot) {
			//print("Digital started, dark: %i \n\r", darkCenter); 
			(*_lastReadingMs)[deviceNumber] = 0;
			for (uint8_t i = 0; i < 8; i++) { // 8 tries
				start(deviceNumber, darkCenter ? 1 : 2); // As digital with dark or bright center
				// Wait for 1. message.
				uint32_t startMs = millis();
				while (millis() - startMs < 50) {
					if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
						//print("Digital confirmed\n\r"); 
						(*_mode)[deviceNumber] = darkCenter ? DIGITAL_AND_DARK_CENTER : DIGITAL_AND_BRIGHT_CENTER;
						return true;
					}
					robotContainer->delayMs(1);
				}
			}
			strcpy(errorMessage, "mrm-ref-can dead.\n\r");
		}
		return false;
	}
	else
		return true;
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - target device found
*/
bool Mrm_ref_can::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
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
					(*_lastReadingMs)[deviceNumber] = millis();
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
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data, false);
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
			}

			return true;
		}
	return false;
}

/** Analog readings
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT) {
		strcpy(errorMessage, "mrm-ref-can doesn't exist");
		return 0;
	}
	alive(deviceNumber, true);
	if (analogStarted(deviceNumber))
		return (*_reading)[deviceNumber][receiverNumberInSensor];
	else
		return 0;
}

/** Print all analog readings in a line
*/
void Mrm_ref_can::readingsPrint() {
	print("Refl:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
			if (alive(deviceNumber))
				print("%3i ", reading(irNo, deviceNumber));
	}
}

/**Test
@param analog - if true, analog values - if not, digital values.
*/
void Mrm_ref_can::test(bool analog)
{
	static uint32_t lastMs = 0;
	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print("| ");
				for (uint8_t i = 0; i < MRM_REF_CAN_SENSOR_COUNT; i++)
					print(analog ? "%3i " : "%i", analog ? reading(i, deviceNumber) : dark(i, deviceNumber));
				if (!analog)
					print(" c:%i", center(deviceNumber, (*_mode)[deviceNumber] == DIGITAL_AND_DARK_CENTER));

			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}


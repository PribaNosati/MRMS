#include "mrm-lid-d.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_d::Mrm_lid_d(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "LidMul", maxNumberOfBoards, ID_MRM_LID_D, 1) {
	readings = new std::vector<std::vector<uint16_t>>(maxNumberOfBoards);
	_resolution = new std::vector<uint8_t>(maxNumberOfBoards);
}

Mrm_lid_d::~Mrm_lid_d()
{
}

/** Add a mrm-lid-can-b device
@param deviceName - device's name
@param resolution - 16 or 64, number of measuring dots
*/
void Mrm_lid_d::add(char * deviceName, uint8_t resolution)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_LID_D_0_IN;
		canOut = CAN_ID_LID_D_0_OUT;
		break;
	case 1:
		canIn = CAN_ID_LID_D_1_IN;
		canOut = CAN_ID_LID_D_1_OUT;
		break;
	case 2:
		canIn = CAN_ID_LID_D_2_IN;
		canOut = CAN_ID_LID_D_2_OUT;
		break;
	case 3:
		canIn = CAN_ID_LID_D_3_IN;
		canOut = CAN_ID_LID_D_3_OUT;
		break;
	case 4:
		canIn = CAN_ID_LID_D_4_IN;
		canOut = CAN_ID_LID_D_4_OUT;
		break;
	case 5:
		canIn = CAN_ID_LID_D_5_IN;
		canOut = CAN_ID_LID_D_5_OUT;
		break;
	case 6:
		canIn = CAN_ID_LID_D_6_IN;
		canOut = CAN_ID_LID_D_6_OUT;
		break;
	case 7:
		canIn = CAN_ID_LID_D_7_IN;
		canOut = CAN_ID_LID_D_7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-lid-d");
		return;
	}
	(*_resolution)[nextFree] = resolution;
	(*readings)[nextFree] = std::vector<uint16_t>(64); //Optionally, resolution instead of 64 but in that case resolution cannot be changed
	SensorBoard::add(deviceName, canIn, canOut);
}


/** Reset sensor's non-volatile memory to defaults (distance mode, timing budget, region of interest, and measurement time, but leaves CAN Bus id intact
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - resets all.
*/
void Mrm_lid_d::defaults(uint8_t deviceNumber) {
	// timingBudget(deviceNumber);
	// robotContainer->delayMs(50); // Allow 50 ms for flash to be written
	// measurementTime(deviceNumber);
	// robotContainer->delayMs(50);
	// distanceMode(deviceNumber);
	// robotContainer->delayMs(50);
	// roi(deviceNumber);
}

/** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t Mrm_lid_d::distance(uint8_t deviceNumber, uint8_t sampleCount, uint8_t sigmaCount){
	const uint16_t TIMEOUT = 3000;
	if (deviceNumber >= nextFree) {
		strcpy(errorMessage, "mrm-lid-d doesn't exist");
		return 0;
	}
	if (started(deviceNumber))
		if (sampleCount == 0)
			return distanceShortest(deviceNumber);
		else{
			uint16_t rds[sampleCount];
			for (uint8_t i = 0; i < sampleCount; i++){
				robotContainer->print("temp: %i\n\r",(*readings)[deviceNumber]);
				if (i != 0) // For 2. reading, etc. - force new readout
					(*readings)[deviceNumber][0] = 0; 
				uint32_t ms = millis();
				while ((*readings)[deviceNumber][0] == 0){
					robotContainer->noLoopWithoutThis();
					if (millis() - ms > TIMEOUT){
						errorCode = 73;
						break;
					}
				}
				rds[i] = distanceShortest(deviceNumber);
				// robotContainer->print("Reading %i\n\r", (*readings)[deviceNumber]);
			}

			// Average and standard deviation
			float mean;
			float sd = stardardDeviation(sampleCount, rds, &mean);

			return outlierlessAverage(sampleCount, rds, mean, sigmaCount, sd);
		}
	else
		return 0;
}


/** Minimum distance in mm. 
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - distance in mm
*/
uint16_t Mrm_lid_d::distanceShortest(uint8_t deviceNumber){
	uint16_t minimum = 0xFFFF;
	for (uint8_t j = 0; j < (*_resolution)[deviceNumber]; j++){
		if ((*readings)[deviceNumber][j] < minimum)
			minimum = (*readings)[deviceNumber][j];
	}
	return minimum;
}

/** Dot distance
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param x - x coordinate
@param y - y coordinate
@return - distance in mm
*/
uint16_t Mrm_lid_d::dot(uint8_t deviceNumber, uint8_t x, uint8_t y){
	if (started(deviceNumber)){
		if (((*_resolution)[deviceNumber] == 16 && (x > 3 || y > 3)) || ((*_resolution)[deviceNumber] == 64 && (x > 7 || y > 7))){
			strcpy(errorMessage, "Coordinate error");
			return 0;
		}
		uint8_t result;
		if ((*_resolution)[deviceNumber] == 16)
			result = (3 - y) * 4 + 3 - x;
		else
			result = (7 - y) * 8 + 7 - x;
		return (*readings)[deviceNumber][result];
	}
	else
		return 0;
}


/** Frequency.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param frequency - up to 60 for 4x4 and 15 for 8x8.
*/
void Mrm_lid_d::frequencySet(uint8_t deviceNumber, uint8_t frequency){
	if ((*_resolution)[deviceNumber] == 16 && frequency > 60 || (*_resolution)[deviceNumber] == 64 && frequency > 15){
		strcpy(errorMessage, "Wrong frequency");
		return;
	}
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			resolutionSet(i, frequency);
	else if (alive(deviceNumber)) {
		delay(1);
		canData[0] = COMMAND_LID_D_FREQUENCY;
		canData[1] = frequency;
		messageSend(canData, 2, deviceNumber);
	}
}


/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_lid_d::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length){
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)){
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint8_t startIndex = data[1];
					for (uint8_t j = 2; j < 7; j+=2){
						uint16_t mm = (data[j+1] << 8) | data[j];
						(*readings)[deviceNumber][startIndex++] = mm;
						// robotContainer->print("Distance for %i: %i mm (%i %i)\n\r", startIndex-1, mm, data[j], data[j+1]); //AAA
					}
					(*_lastReadingMs)[deviceNumber] = millis();
				}
				break;
				case COMMAND_INFO_SENDING_1:
					robotContainer->print("%s: %s dist., budget %i ms, %ix%i, intermeas. %i ms\n\r", name(deviceNumber), data[1] ? "short" : "long", data[2] | (data[3] << 8),
						data[4] & 0xFF, data[5] & 0xFF, data[6] | (data[7] << 8));
					break;
				default:
					robotContainer->print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 202;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}

/** Enable plug and play
@param enable - enable or disable
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_d::pnpSet(bool enable, uint8_t deviceNumber){
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			pnpSet(enable, i);
	else if (alive(deviceNumber)) {
		delay(1);
		canData[0] = enable ? COMMAND_LID_D_PNP_ENABLE : COMMAND_LID_D_PNP_DISABLE;
		canData[1] = enable;
		messageSend(canData, 2, deviceNumber);
	}
}

/** Analog readings
@param receiverNumberInSensor - always 0
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_d::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	return distance(deviceNumber);
}

/** Print all readings in a line
*/
void Mrm_lid_d::readingsPrint() {
	robotContainer->print("Lid4mMul:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			robotContainer->print(" %4i", distance(deviceNumber));
}

/** Resolution, 4x4 or 8x8.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param resolution - 16 or 64. Default 16.
*/
void Mrm_lid_d::resolutionSet(uint8_t deviceNumber, uint8_t resolution){
	if (resolution != 16 && resolution != 64){
		strcpy(errorMessage, "Wrong resolution");
		return;
	}
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			resolutionSet(i, resolution);
	else if (alive(deviceNumber)) {
		delay(1);
		canData[0] = COMMAND_LID_D_RESOLUTION;
		canData[1] = resolution;
		messageSend(canData, 2, deviceNumber);
		(*_resolution)[deviceNumber] = resolution;
	}
}


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_lid_d::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_LID_D_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//robotContainer->print("Start mrm-lid-can-b2%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//robotContainer->print("Lidar confirmed\n\r"); 
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-lid-d dead.\n\r");
		return false;
	}
	else
		return true;
}


/**Test
*/
void Mrm_lid_d::test()
{
	static uint32_t lastMs = 0;
#define MRM_LID_H_TEST_MULTI 1	
#define MRM_LID_H_TEST_64 0
	if (robotContainer->setup())
#if MRM_LID_H_TEST_64
		resolutionSet(0xFF, 64);
#else
		resolutionSet(0xFF, 16);
#endif
	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
#if MRM_LID_H_TEST_MULTI
				for (int8_t y = ((*_resolution)[deviceNumber] == 64 ? 7 : 3); y >= 0; y--){
					for (uint8_t x = 0; x < ((*_resolution)[deviceNumber] == 64 ? 8 : 4); x++)
						robotContainer->print("%4i ", dot(deviceNumber, x, y));
					robotContainer->print("\n\r");
				}
#endif
				if (pass++)
					robotContainer->print(" ");
#if !MRM_LID_H_TEST_MULTI
				robotContainer->print("%i ", distance(deviceNumber));
#endif
			}
		}
		lastMs = millis();
		if (pass)
			robotContainer->print("\n\r");
	}
}
#include "mrm-lid-can-b.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_can_b::Mrm_lid_can_b(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Lid2m", maxNumberOfBoards, ID_MRM_LID_CAN_B, 1) {
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
		strcpy(errorMessage, "Too many mrm-lid-can-b");
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
	else if (alive(deviceNumber)){
		canData[0] = COMMAND_LID_CAN_B_CALIBRATE;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 1, canData);
	}
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
uint16_t Mrm_lid_can_b::distance(uint8_t deviceNumber, uint8_t sampleCount, uint8_t sigmaCount){
	const uint16_t TIMEOUT = 3000;
	if (deviceNumber > nextFree) {
		strcpy(errorMessage, "mrm-lid-can-b doesn't exist");
		return 0;
	}
	alive(deviceNumber, true); // This command doesn't make sense
	if (started(deviceNumber)){
		if (sampleCount == 0)
			return (*readings)[deviceNumber];
		else{
			uint16_t rds[sampleCount];
			for (uint8_t i = 0; i < sampleCount; i++){
				if (i != 0) // For 2. reading, etc. - force new readout
					(*readings)[deviceNumber] = 0;
				uint32_t ms = millis();
				while ((*readings)[deviceNumber] == 0){
					robotContainer->noLoopWithoutThis();
					if (millis() - ms > TIMEOUT){
						errorCode = 73;
						break;
					}
				}
				rds[i] = (*readings)[deviceNumber];
				//robotContainer->print("Reading %i\n\r", (*readings)[deviceNumber]);
			}

			// Average and standard deviation
			float sum = 0.0;
			for(uint8_t i = 0; i < sampleCount; i++)
				sum += rds[i];
			//robotContainer->print("Sum %i\n\r", (int)sum);
			float mean = sum / sampleCount;
			//robotContainer->print("Mean %i\n\r", (int)mean);
			float standardDeviation = 0.0;
			for(int i = 0; i < sampleCount; i++) 
				standardDeviation += pow(rds[i] - mean, 2);
			standardDeviation = sqrt(standardDeviation / sampleCount);
			//robotContainer->print("SD %i\n\r", (int)standardDeviation);

			// Filter out all the values outside n-sigma boundaries and return average value of the rest
			sum = 0;
			uint8_t cnt = 0;
			//robotContainer->print("Limits: %i %i (%i)\n\r", (int)(mean - sigmaCount * standardDeviation), (int)(mean + sigmaCount * standardDeviation), sigmaCount);
			for (uint8_t i = 0; i < sampleCount; i++)
				if (mean - sigmaCount * standardDeviation < rds[i] && rds[i] < mean + sigmaCount * standardDeviation){
					sum += rds[i];
					cnt++;
				}

			//robotContainer->print("Cnt %i\n\r", cnt);
			return (uint16_t)(sum / cnt);
		}
	}
	else
		return 0;
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_lid_can_b::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length){
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint16_t mm = (data[2] << 8) | data[1];
					(*readings)[deviceNumber] = mm;
					(*_lastReadingMs)[deviceNumber] = millis();
				}
				break;
				default:
					robotContainer->print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 206;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	}
	return false;
}

/** Enable plug and play
@param enable - enable or disable
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_can_b::pnpSet(bool enable, uint8_t deviceNumber){
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			pnpSet(enable, i);
	else if (alive(deviceNumber)) {
		delay(1);
		canData[0] = enable ? COMMAND_LID_CAN_B_PNP_ENABLE : COMMAND_LID_CAN_B_PNP_DISABLE;
		canData[1] = enable;
		messageSend(canData, 2, deviceNumber);
	}
}

/** Ranging type
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param value - long range 0, high speed 1, high accuracy 2
*/
void Mrm_lid_can_b::rangingType(uint8_t deviceNumber, uint8_t value) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibration(i);
	else {
		canData[0] = COMMAND_LID_CAN_B_RANGING_TYPE;
		canData[1] = value;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
	}
}

/** Analog readings
@param receiverNumberInSensor - always 0
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_can_b::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	return distance(deviceNumber);
}

/** Print all readings in a line
*/
void Mrm_lid_can_b::readingsPrint() {
	robotContainer->print("Lid2m:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber))
			robotContainer->print(" %4i", distance(deviceNumber));
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_lid_can_b::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_LID_CAN_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		robotContainer->print("Start mrm-lid-can-b%i \n\r", deviceNumber); 
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 10) {
				//robotContainer->print("-try-");
				if (millis() - (*_lastReadingMs)[deviceNumber] < 20) {
					//robotContainer->print("%s confirmed\n\r", (char*)name(deviceNumber));
					return true;
				}
				robotContainer->delayMicros(1000);
			}
		}
		sprintf(errorMessage, "%s not responding.\n\r", (char*)name(deviceNumber)); // To be reported later.
		robotContainer->print(errorMessage);
		return false;
	}
	else
		return true;
}


/**Test
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param betweenTestsMs - time in ms between 2 tests. 0 - default.
*/
void Mrm_lid_can_b::test(uint8_t deviceNumber, uint16_t betweenTestsMs)
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > (betweenTestsMs == 0 ? 300 : betweenTestsMs)) {
		uint8_t pass = 0;
		for (uint8_t i = 0; i < nextFree; i++) {
			bool isAlive = alive(i);
			// robotContainer->print("L%i:%s", i, isAlive ? "Y" : "N"); 
			if (isAlive && (deviceNumber == 0xFF || i == deviceNumber)) {
				if (pass++)
					robotContainer->print(" ");
				robotContainer->print("%4i ", distance(i));
			}
		}
		lastMs = millis();
		if (pass)
			robotContainer->print("\n\r");
	}
}
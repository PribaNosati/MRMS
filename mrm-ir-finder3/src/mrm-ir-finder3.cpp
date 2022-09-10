#include "mrm-ir-finder3.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_ir_finder3::Mrm_ir_finder3(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "IRFind3", maxNumberOfBoards, ID_MRM_IR_FINDER3, MRM_IR_FINDER3_SENSOR_COUNT) {
	_angle = new std::vector<int16_t>(maxNumberOfBoards);
	_calculated = new std::vector<bool>(maxNumberOfBoards);
	_distance = new std::vector<uint16_t>(maxNumberOfBoards);
	_near = new std::vector<bool>(maxNumberOfBoards);
	readings = new std::vector<uint16_t[MRM_IR_FINDER3_SENSOR_COUNT]>(maxNumberOfBoards);
	measuringModeLimit = 2;
}

Mrm_ir_finder3::~Mrm_ir_finder3()
{
}

/** Add a mrm-ir-finder3 sensor
@param deviceName - device's name
*/
void Mrm_ir_finder3::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = ID_IR_FINDER3_0_IN;
		canOut = ID_IR_FINDER3_0_OUT;
		break;
	case 1:
		canIn = ID_IR_FINDER3_1_IN;
		canOut = ID_IR_FINDER3_1_OUT;
		break;
	case 2:
		canIn = ID_IR_FINDER3_2_IN;
		canOut = ID_IR_FINDER3_2_OUT;
		break;
	case 3:
		canIn = ID_IR_FINDER3_3_IN;
		canOut = ID_IR_FINDER3_3_OUT;
		break;
	case 4:
		canIn = ID_IR_FINDER3_4_IN;
		canOut = ID_IR_FINDER3_4_OUT;
		break;
	case 5:
		canIn = ID_IR_FINDER3_5_IN;
		canOut = ID_IR_FINDER3_5_OUT;
		break;
	case 6:
		canIn = ID_IR_FINDER3_6_IN;
		canOut = ID_IR_FINDER3_6_OUT;
		break;
	case 7:
		canIn = ID_IR_FINDER3_7_IN;
		canOut = ID_IR_FINDER3_7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-ir-finder3");
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}


/** Ball's direction
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - robot's front is 0�, positive angles clockwise, negative anti-clockwise. Back of the robot is 180�.
*/
int16_t Mrm_ir_finder3::angle(uint8_t deviceNumber) {
	if (calculatedStarted(deviceNumber))
		return (*_angle)[deviceNumber];
	else
		return 0;
}

/** If calculated mode not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_ir_finder3::calculatedStarted(uint8_t deviceNumber) {
	if (!(*_calculated)[deviceNumber] || millis() - (*_lastReadingMs)[deviceNumber] > MRM_IR_FINDER3_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		robotContainer->print("Start IR finder \n\r"); 
		(*_lastReadingMs)[deviceNumber] = 0;
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 1); // As calculated
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					robotContainer->print("IR3 confirmed\n\r"); 
					(*_calculated)[deviceNumber] = true;
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-ir-finder3 dead.\n\r");
		return false;
	}
	else
		return true;
}

/** Ball's distance
@return - this is analog value that represents infrared light intensity, so not directly distance, but the distance can be inferred. When ball is quite close, expect values up to about 3000.
	At about 1 m is the boundary between 2 zones so the value will drop sharply as long-dinstance sensors engage.
	When 0 is return, there is no ball in sight.
*/
uint16_t Mrm_ir_finder3::distance(uint8_t deviceNumber) {
	if (calculatedStarted(deviceNumber))
		return (*_distance)[deviceNumber];
	else
		return 0;
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_ir_finder3::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length) {
	// Todo: a problem: one message can be for short range sensors, the other for long. A mixed data will be the result.
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				bool any = false;
				uint8_t startIndex = 0;
				uint8_t length = 7;
				switch (data[0]) {
				case COMMAND_IR_FINDER3_SENDING_SENSORS_1_TO_7:
					any = true;
					break;
				case COMMAND_IR_FINDER3_SENDING_SENSORS_8_TO_12:
					startIndex = 7;
					length = 5;
					(*_near)[deviceNumber] = data[6];
					(*_lastReadingMs)[deviceNumber] = millis();
					any = true;
					break;
				case COMMAND_SENSORS_MEASURE_CALCULATED_SENDING:
					(*_angle)[deviceNumber] = ((data[1] << 8) | data[2]) - 180;
					(*_distance)[deviceNumber] = (data[3] << 8) | data[4];
					(*_near)[deviceNumber] = data[5];
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				default:
					robotContainer->print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 201;
					errorInDeviceNumber = deviceNumber;
				}

				if (any)
					for (uint8_t i = 0; i < length; i++)
						(*readings)[deviceNumber][startIndex + i] = data[i + 1];
			}
			return true;
		}
	return false;
}

/** Analog readings
@param receiverNumberInSensor - single IR receiver in mrm-ir-finder3
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_ir_finder3::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_IR_FINDER3_SENSOR_COUNT) {
		strcpy(errorMessage, "mrm-ir-finder3 doesn't exist");
		return 0;
	}
	if (singleStarted(deviceNumber))
		return (*readings)[deviceNumber][receiverNumberInSensor];
	else
		return 0;
}

/** Print all readings in a line
*/
void Mrm_ir_finder3::readingsPrint() {
	robotContainer->print("IRBall:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (alive(deviceNumber)) {
			for (uint8_t irNo = 0; irNo < MRM_IR_FINDER3_SENSOR_COUNT; irNo++)
				robotContainer->print(" %3i", reading(irNo, deviceNumber));
		}
}

/**Test
*/
void Mrm_ir_finder3::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					robotContainer->print("| ");
				uint8_t last;
				if ((*_near)[deviceNumber]){
					last = MRM_IR_FINDER3_SENSOR_COUNT;
					robotContainer->print("Near ");
				}
				else{
					last = MRM_IR_FINDER3_SENSOR_COUNT / 2;
					robotContainer->print("Far ");
				}
				for (uint8_t i = 0; i < last; i++)
					robotContainer->print("%i ", reading(i, deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			robotContainer->print("\n\r");
	}
}

/** If single mode not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_ir_finder3::singleStarted(uint8_t deviceNumber) {
	if ((*_calculated)[deviceNumber] || millis() - (*_lastReadingMs)[deviceNumber] > MRM_IR_FINDER3_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		robotContainer->print("Start IR finder \n\r"); 
		(*_lastReadingMs)[deviceNumber] = 0;
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0); // As single
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					robotContainer->print("IR3 confirmed\n\r"); 
					(*_calculated)[deviceNumber] = false;
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-ir-finder3 dead.\n\r");
		return false;
	}
	else
		return true;
}

/**Test
*/
void Mrm_ir_finder3::testCalculated()
{
	static uint32_t lastMs = 0;
	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) 
				robotContainer->print("%s: %i deg., dist: %i\n\r", (*_near)[deviceNumber] ? "Near" : "Far",
				angle(), distance());
		}
		lastMs = millis();
		if (pass)
			robotContainer->print("\n\r");
	}
}


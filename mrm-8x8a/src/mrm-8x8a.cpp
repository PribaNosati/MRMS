#include "mrm-8x8a.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_8x8a::Mrm_8x8a(Robot* robot, uint8_t maxNumberOfBoards) : SensorBoard(robot, 1, "LED8x8", maxNumberOfBoards, ID_MRM_8x8A) {
	displayedLast = new std::vector<uint8_t>(maxNumberOfBoards);
	displayedTypeLast = new std::vector<uint8_t>(maxNumberOfBoards);
	lastOn = new std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>(maxNumberOfBoards);
	on = new std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>(maxNumberOfBoards);
	offOnAction = new std::vector<ActionBase* [MRM_8x8A_SWITCHES_COUNT]>(maxNumberOfBoards);
	//mrm_can_bus = esp32CANBusSingleton;
	nextFree = 0;
}

Mrm_8x8a::~Mrm_8x8a()
{
}


ActionBase* Mrm_8x8a::actionCheck() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) { // AAA 
		for (uint8_t switchNumber = 0; switchNumber < MRM_8x8A_SWITCHES_COUNT; switchNumber++)
			if ((*lastOn)[deviceNumber][switchNumber] == false && switchRead(switchNumber, deviceNumber) && (*offOnAction)[deviceNumber][switchNumber] != NULL)
				return (*offOnAction)[deviceNumber][switchNumber];
			else if ((*lastOn)[deviceNumber][switchNumber] == true && !switchRead(switchNumber, deviceNumber))
				((*lastOn)[deviceNumber][switchNumber]) = false;
	}
	return NULL;
}

void Mrm_8x8a::actionSet(ActionBase* action, uint8_t switchNumber, uint8_t deviceNumber) {
	(*offOnAction)[deviceNumber][switchNumber] = action;
}

/** Add a mrm-8x8a board
@param deviceName - device's name
*/
void Mrm_8x8a::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_8x8A0_IN;
		canOut = CAN_ID_8x8A0_OUT;
		break;
	case 1:
		canIn = CAN_ID_8x8A1_IN;
		canOut = CAN_ID_8x8A1_OUT;
		break;
	case 2:
		canIn = CAN_ID_8x8A2_IN;
		canOut = CAN_ID_8x8A2_OUT;
		break;
	case 3:
		canIn = CAN_ID_8x8A3_IN;
		canOut = CAN_ID_8x8A3_OUT;
		break;
	case 4:
		canIn = CAN_ID_8x8A4_IN;
		canOut = CAN_ID_8x8A4_OUT;
		break;
	case 5:
		canIn = CAN_ID_8x8A5_IN;
		canOut = CAN_ID_8x8A5_OUT;
		break;
	case 6:
		canIn = CAN_ID_8x8A6_IN;
		canOut = CAN_ID_8x8A6_OUT;
		break;
	case 7:
		canIn = CAN_ID_8x8A7_IN;
		canOut = CAN_ID_8x8A7_OUT;
		break;
	default:
		strcpy(errorMessage, "Too many mrm-8x8a");
	}

	for (uint8_t i = 0; i < MRM_8x8A_SWITCHES_COUNT; i++) {
		(*on)[nextFree][i] = false;
		(*lastOn)[nextFree][i] = false;
		(*offOnAction)[nextFree][i] = NULL;
	}

	(*displayedLast)[nextFree] = 0xFF;
	(*displayedTypeLast)[nextFree] = LED8x8Type::LED_8X8_CUSTOM;

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Display stored (in sensor, read-only) bitmap
@param bitmapId - bitmap's id
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapDisplay(uint8_t bitmapId, uint8_t deviceNumber){
	if (bitmapId != (*displayedLast)[deviceNumber] || (*displayedTypeLast)[deviceNumber] != LED8x8Type::LED_8X8_STORED) {
		(*displayedLast)[deviceNumber] = bitmapId;
		(*displayedTypeLast)[deviceNumber] = LED8x8Type::LED_8X8_STORED;
		alive(deviceNumber, true);
		canData[0] = COMMAND_8X8_DISPLAY;
		canData[1] = bitmapId;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
	}
}

/** Display custom bitmap
@param red - 8-byte array for red
@param green - 8-byte array for green
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapCustomDisplay(uint8_t red[], uint8_t green[], uint8_t deviceNumber) {
	alive(deviceNumber, true);
	canData[0] = COMMAND_8X8_BITMAP_DISPLAY_PART1;
	for (uint8_t i = 0; i < 7; i++) 
		canData[i + 1] = green[i];
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 8, canData);

	canData[0] = COMMAND_8X8_BITMAP_DISPLAY_PART2;
	canData[1] = green[7];
	for (uint8_t i = 0; i < 6; i++) 
		canData[i + 2] = red[i];
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 8, canData);

	canData[0] = COMMAND_8X8_BITMAP_DISPLAY_PART3;
	for (uint8_t i = 0; i < 2; i++) 
		canData[i + 1] = red[i + 6];
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 3, canData);

	(*displayedTypeLast)[deviceNumber] = LED8x8Type::LED_8X8_CUSTOM;
}

/** Store custom bitmap
@param red - 8-byte array for red
@param green - 8-byte array for green
@param addres - address in display's RAM. 0 - 99.
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapCustomStore(uint8_t red[], uint8_t green[], uint8_t address, uint8_t deviceNumber) {
	alive(deviceNumber, true);

	canData[0] = COMMAND_8X8_BITMAP_STORE_PART1;
	for (uint8_t i = 0; i < 7; i++)
		canData[i + 1] = green[i];
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 8, canData);

	canData[0] = COMMAND_8X8_BITMAP_STORE_PART2;
	canData[1] = green[7];
	for (uint8_t i = 0; i < 6; i++)
		canData[i + 2] = red[i];
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 8, canData);

	canData[0] = COMMAND_8X8_BITMAP_STORE_PART3;
	for (uint8_t i = 0; i < 2; i++)
		canData[i + 1] = red[i + 6];
	canData[3] = address;
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 4, canData);
}

/** Display custom stored bitmap
@param addres - address in display's RAM. 0 - 99.
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapCustomStoredDisplay(uint8_t address, uint8_t deviceNumber) {
	if (address != (*displayedLast)[deviceNumber] || (*displayedTypeLast)[deviceNumber] != LED8x8Type::LED_8X8_STORED_CUSTOM) {
		(*displayedLast)[deviceNumber] = address;
		(*displayedTypeLast)[deviceNumber] = LED8x8Type::LED_8X8_STORED_CUSTOM;
		alive(deviceNumber, true);
		canData[0] = COMMAND_8X8_BITMAP_STORED_DISPLAY;
		canData[1] = address;
		robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
	}
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_8x8a::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_8X8_SWITCH_ON:
				case COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION: {
					uint8_t switchNumber = data[1] >> 1;
					if (switchNumber > 4) {
						strcpy(errorMessage, "No 8x8a switch");
						return false;
					}
					(*on)[deviceNumber][switchNumber] = data[1] & 1;
					if (data[0] == COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION) {
						canData[0] = COMMAND_NOTIFICATION;
						canData[1] = switchNumber; //todo - deviceNumber not taken into account
						robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
					}
					(*_lastReadingMs)[deviceNumber] = millis();
				}
					break;
				case COMMAND_8x8_TEST_CAN_BUS:
					print("Test: %i\n\r", data[1]);
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, 8, data, false);
					print("\n\r");
					errorCode = 203;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}

/** Displays 8-row progress bar. Useful for visual feedback of a long process.
@param period - total count (100%)
@param current - current count (current percentage)
@param reset - reset to start (no bar)
@return - display changed
*/
bool Mrm_8x8a::progressBar(uint32_t period, uint32_t current, bool reset) {
	static uint32_t _period = 0;
	static uint32_t lastDisplayMs = 0;
	static uint8_t lastDot = 0xFF;
	static bool lastGreen = true;

	if (current > period) {
		strcpy(errorMessage, "Overflow in progress bar");
		return false;
	}

	if (reset) {
		_period = period;
		lastDisplayMs = 0;
	}

	uint8_t currentDot = ceil(current * 64 / (float)period); // 0 - 64
	if (millis() - lastDisplayMs > 500 || currentDot != lastDot) { // Blink each 0.5 sec
		uint8_t green[8], red[8];
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0b00000000;

		// Display full rows.
		uint8_t partialRow = currentDot / 8;
		for (uint8_t i = 0; i < partialRow; i++) // Display full rows, 0 - 8
			green[i] = 0b11111111;

		// Display partially full row.
		for (uint8_t i = 0; i < 8; i++)
			green[partialRow] = (green[partialRow] << 1) | (i < currentDot - partialRow * 8 ? 1 : 0);

		// Display empty rows.
		for (uint8_t i = partialRow + 1; i < 8; i++)
			green[i] = 0b00000000;

		bitmapCustomDisplay(lastGreen ? red : green, lastGreen ? green : red);
		lastDot = currentDot;
		lastDisplayMs = millis();
		lastGreen = !lastGreen; // todo
		return true;
	}
	else
		return false;
}

/** Set rotation from now on
@param rotation - 0, 90, or 270 degrees counterclockwise
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::rotationSet(enum LED8x8Rotation rotation, uint8_t deviceNumber) {
	alive(deviceNumber, true);
	canData[0] = COMMAND_8X8_ROTATION_SET;
	canData[1] = rotation;
	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_8x8a::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_8X8A_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//print("Start mrm-8x8a%i \n\r", deviceNumber); 
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//print("8x8 confirmed\n\r"); 
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-8x8a dead.\n\r");
		return false;
	}
	else
		return true;
}

/** Read switch
@param switchNumber
@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if pressed, false otherwise
*/
bool Mrm_8x8a::switchRead(uint8_t switchNumber, uint8_t deviceNumber) {
	alive(deviceNumber, true);
	if (deviceNumber >= nextFree || switchNumber >= MRM_8x8A_SWITCHES_COUNT) {
		strcpy(errorMessage, "Switch doesn't exist");
		return false;
	}
	if (started(deviceNumber))
		return (*on)[deviceNumber][switchNumber];
	else
		return false;
}


/**Test
*/
void Mrm_8x8a::test()
{
#define MRM_8x8A_START_BITMAP_1 0x01
#define MRM_8x8A_END_BITMAP_1 0x04
#define MRM_8x8A_START_BITMAP_2 0x30
#define MRM_8x8A_END_BITMAP_2 0x5A
	static uint32_t lastMs = 0;
	static uint8_t bitmapId = MRM_8x8A_START_BITMAP_1;

	if (robotContainer->setup()) {
		uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
		uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
		bitmapCustomStore(red, green, 7);
	}

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;

		// Built-in bitmap
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				bitmapDisplay(bitmapId, deviceNumber);
				if (pass++)
					print("| ");
				print("Map 0x%02x, sw:", bitmapId);
				for (uint8_t i = 0; i < MRM_8x8A_SWITCHES_COUNT; i++)
					print("%i ", switchRead(i, deviceNumber));
			}
		}
		bitmapId++;
		if (bitmapId > MRM_8x8A_END_BITMAP_1 && bitmapId < MRM_8x8A_START_BITMAP_2)
			bitmapId = MRM_8x8A_START_BITMAP_2;
		else if (bitmapId > MRM_8x8A_END_BITMAP_2) {
			bitmapId = MRM_8x8A_START_BITMAP_1;

			// Custom bitmap.
			robotContainer->delayMs(300);
			uint8_t red[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000100, 0b00111000, 0b00000000, 0b00111100 };
			uint8_t green[8] = { 0b00111100, 0b01000010, 0b10101001, 0b10101001, 0b10000001, 0b10000001, 0b01000010, 0b00111100 };
			bitmapCustomDisplay(red, green);

			// Custom stored bitmap
			robotContainer->delayMs(300);
			bitmapCustomStoredDisplay(7);
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

/** Display text
@param content - text
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::text(char content[], uint8_t deviceNumber) {
	uint8_t message = 0;
	bool unsent = false;
	for (uint8_t i = 0; i < MRM_8X8A_TEXT_LENGTH; i++) {
		if (i % 7 == 0 && i != 0) {
			canData[0] = COMMAND_8X8_TEXT_1 + message;
			messageSend(canData, 8, deviceNumber);
			robotContainer->delayMs(1);
			message++;
			unsent = false;
		}
		canData[i % 7 + 1] = content[i];
		unsent = true;
		if (content[i] == '\0')
			break;
	}
	if (unsent) {
		canData[0] = COMMAND_8X8_TEXT_1 + message;
		messageSend(canData, 8, deviceNumber);
	}
}


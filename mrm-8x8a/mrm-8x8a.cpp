#include "mrm-8x8a.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;
extern char* errorMessage;

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_8x8a::Mrm_8x8a(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial, uint8_t maxDevices) : SensorBoard(esp32CANBusSingleton, 1, "LED8x8", maxDevices) {
	lastOn = new std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>(maxDevices);
	on = new std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>(maxDevices);
	offOnAction = new std::vector<Command* [MRM_8x8A_SWITCHES_COUNT]>(maxDevices);
	esp32CANBus = esp32CANBusSingleton;
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_8x8a::~Mrm_8x8a()
{
}


Command* Mrm_8x8a::actionCheck() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t switchNumber = 0; switchNumber < MRM_8x8A_SWITCHES_COUNT; switchNumber++)
			if ((*lastOn)[deviceNumber][switchNumber] == false && (*on)[deviceNumber][switchNumber] == true && (*offOnAction)[deviceNumber][switchNumber] != NULL) {
				((*lastOn)[deviceNumber][switchNumber]) = true;
				return (*offOnAction)[deviceNumber][switchNumber];
			} else if ((*lastOn)[deviceNumber][switchNumber] == true && (*on)[deviceNumber][switchNumber] == false)
				((*lastOn)[deviceNumber][switchNumber]) = false;
	}
	return NULL;
}

void Mrm_8x8a::actionSet(Command* action, uint8_t switchNumber, uint8_t deviceNumber) {
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

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Display bitmap
@param bitmapId - bitmap's id
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapDisplay(uint8_t bitmapId, uint8_t deviceNumber){
	canData[0] = COMMAND_8X8_DISPLAY;
	canData[1] = bitmapId;
	esp32CANBus->messageSend((*idIn)[deviceNumber], 2, canData);
}

/** Display custom bitmap
@param red - 8-byte array for red
@param green - 8-byte array for green
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapDisplayCustom(uint8_t red[], uint8_t green[], uint8_t deviceNumber) {
	canData[0] = COMMAND_8X8_BITMAP_PART1;
	for (uint8_t i = 0; i < 7; i++)
		canData[i + 1] = red[i];
	esp32CANBus->messageSend((*idIn)[deviceNumber], 8, canData);

	canData[0] = COMMAND_8X8_BITMAP_PART2;
	canData[1] = red[7];
	for (uint8_t i = 0; i < 6; i++)
		canData[i + 1] = green[i];
	esp32CANBus->messageSend((*idIn)[deviceNumber], 8, canData);

	canData[0] = COMMAND_8X8_BITMAP_PART3;
	for (uint8_t i = 0; i < 5; i++)
		canData[i + 1] = green[i + 2];
	esp32CANBus->messageSend((*idIn)[deviceNumber], 6, canData);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_8x8a::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			switch (data[0]) {
			case COMMAND_8X8_SWITCH_ON: 
			case COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION:{
				uint8_t switchNumber = data[1] >> 1;
				if (switchNumber > 4) {
					strcpy(errorMessage, "No 8x8a switch");
					return false;
				}
				(*on)[deviceNumber][switchNumber] = data[1] & 1;
				if (data[0] == COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION) {
					canData[0] = COMMAND_NOTIFICATION;
					canData[1] = switchNumber; //todo - deviceNumber not taken into account
					esp32CANBus->messageSend((*idIn)[deviceNumber], 2, canData);
				}
			}
			break;
			case COMMAND_8x8_TEST_CAN_BUS:
				print("Test: %i\n\r", data[1]);
				break;
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
				break;
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 203;
				errorInDeviceNumber = deviceNumber;
			}
			return true;
		}
	return false;
}

/** Read switch
@param switchNumber
@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if pressed, false otherwise
*/
bool Mrm_8x8a::switchRead(uint8_t switchNumber, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || switchNumber >= MRM_8x8A_SWITCHES_COUNT) {
		strcpy(errorMessage, "Switch doesn't exist");
		return false;
	}
	return (*on)[deviceNumber][switchNumber];
}


/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_8x8a::test(BreakCondition breakWhen)
{
#define MRM_8x8A_START_BITMAP_1 0x01
#define MRM_8x8A_END_BITMAP_1 0x04
#define MRM_8x8A_START_BITMAP_2 0x30
#define MRM_8x8A_END_BITMAP_2 0x5A
	static uint32_t lastMs = 0;
	static uint8_t bitmapId = MRM_8x8A_START_BITMAP_1;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				bitmapDisplay(bitmapId, deviceNumber);
				if (pass++)
					print("| ");
				print("Map 0x%02x, sw:", bitmapId);
				for (uint8_t i = 0; i < MRM_8x8A_SWITCHES_COUNT; i++)
					print("%i ", (*on)[deviceNumber][i]);
			}
		}
		lastMs = millis();
		bitmapId++;
		if (bitmapId > MRM_8x8A_END_BITMAP_1 && bitmapId < MRM_8x8A_START_BITMAP_2)
			bitmapId = MRM_8x8A_START_BITMAP_2;
		else if (bitmapId > MRM_8x8A_END_BITMAP_2)
			bitmapId = MRM_8x8A_START_BITMAP_1;
		if (pass)
			print("\n\r");
	}
}


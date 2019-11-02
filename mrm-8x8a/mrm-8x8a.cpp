#include "mrm-8x8a.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_8x8a::Mrm_8x8a(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial, uint8_t maxDevices) : SensorBoard(esp32CANBusSingleton, 1, "LED8x8", maxDevices) {
	on = new std::vector<bool>(maxDevices);
	esp32CANBus = esp32CANBusSingleton;
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_8x8a::~Mrm_8x8a()
{
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
		error("Too many mrm-8x8as\n\r");
	}
	SensorBoard::add(deviceName, canIn, canOut);

	(*on)[nextFree-1] = false;
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

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_8x8a::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			switch (data[0]) {
			case COMMAND_8X8_SWITCH_ON: {
				bool isOn = data[1];
				(*on)[deviceNumber] = isOn;
			}
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
				//error("8x8Deco");
			}
			return true;
		}
	return false;
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_8x8a::test(BreakCondition breakWhen)
{
	uint8_t bitmapId = 0;
	while (!breakWhen()) {
		print("Map %i\n\r", bitmapId);
		bitmapDisplay(bitmapId);
		if (++bitmapId > 3)
			bitmapId = 0;
		delay(500);
	}
	print("\n\rTest over.\n\r");
}


#include "mrm-ref-can.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_ref_can::Mrm_ref_can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) : SensorGroup(esp32CANBusSingleton, 1, "ReflArray") {
	serial = hardwareSerial;
}

Mrm_ref_can::~Mrm_ref_can()
{
}

/** Add a mrm-ref-can sensor
@param deviceName - device's name
*/
void Mrm_ref_can::add(char * deviceName)
{
	SensorGroup::add(deviceName, CAN_ID_REF_CAN0_IN, CAN_ID_REF_CAN0_OUT, CAN_ID_REF_CAN1_IN, CAN_ID_REF_CAN1_OUT,
		CAN_ID_REF_CAN2_IN, CAN_ID_REF_CAN2_OUT, CAN_ID_REF_CAN3_IN, CAN_ID_REF_CAN3_OUT, CAN_ID_REF_CAN4_IN,
		CAN_ID_REF_CAN4_OUT, CAN_ID_REF_CAN5_IN, CAN_ID_REF_CAN5_OUT, CAN_ID_REF_CAN6_IN, CAN_ID_REF_CAN6_OUT,
		CAN_ID_REF_CAN7_IN, CAN_ID_REF_CAN7_IN);
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
		esp32CANBus->messageSend(idIn[deviceNumber], 1, canData);
	}
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_ref_can::messageDecode(uint32_t canId, uint8_t data[8]) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)) {
			bool any = false;
			uint8_t startIndex = 0;
			switch (data[0]) {
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, nameThis[deviceNumber]);
				break;
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3:
				startIndex = 0;
				any = true;
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6:
				startIndex = 3;
				any = true;
				break;
			case COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9:
				startIndex = 6;
				any = true;
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				error("RefDeco");
			}

			if (any)
				for (uint8_t i = 0; i <= 2; i++)
					readings[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

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
	if (deviceNumber > MAX_SENSORS_BASE || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT)
		error("Device doesn't exist");
	return readings[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_ref_can::readingsPrint() {
	print("Refl:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
			if (alive(deviceNumber))
				print(" %3i", readings[deviceNumber][irNo]);
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
					print("%i ", readings[deviceNumber][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}

	//continuousReadingStop();
}


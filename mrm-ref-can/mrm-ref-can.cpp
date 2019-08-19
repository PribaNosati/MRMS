#include "mrm-ref-can.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_ref_can::Mrm_ref_can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) {
	esp32CANBus = esp32CANBusSingleton;
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_ref_can::~Mrm_ref_can()
{
}

/** Add a mrm-ref-can sensor
@param deviceName - device's name
*/
void Mrm_ref_can::add(char * deviceName)
{
	if (nextFree >= MAX_MRM_REF_CAN)
		error("Too many Mrm_ref_can");

	switch (nextFree) {
	case 0:
		idIn[nextFree] = CAN_ID_REF_CAN0_IN;
		idOut[nextFree] = CAN_ID_REF_CAN0_OUT;
		break;
	case 1:
		idIn[nextFree] = CAN_ID_REF_CAN1_IN;
		idOut[nextFree] = CAN_ID_REF_CAN1_IN;
		break;
	case 2:
		idIn[nextFree] = CAN_ID_REF_CAN2_IN;
		idOut[nextFree] = CAN_ID_REF_CAN2_IN;
		break;
	case 3:
		idIn[nextFree] = CAN_ID_REF_CAN3_IN;
		idOut[nextFree] = CAN_ID_REF_CAN3_IN;
		break;
	default:
		error("Too many sensors.");
	}

	if (deviceName != 0) {
		if (strlen(deviceName) > 9)
			error("Name too long");
		strcpy(nameThis[nextFree], deviceName);
	}
	nextFree++;
}

/** Calibrate the array
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - calibrate all sensors.
*/
void Mrm_ref_can::calibrate(uint8_t sensorNumber) {
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			calibrate(i);
	else {
		uint8_t data = COMMAND_REFLECTANCE_ARRAY_CALIBRATE;
		esp32CANBus->messageSend(idIn[sensorNumber], 1, &data);
	}
}

/** Starts periodical CAN Bus messages that will be refreshing values that can be read by reading()
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_ref_can::continuousReadingStart(uint8_t sensorNumber){
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStart(i);
	else{
		uint8_t data[8] = { COMMAND_REFLECTANCE_ARRAY_MEASURE_CONTINUOUS_EACH };
		esp32CANBus->messageSend(idIn[sensorNumber], 1, data);
	}
}

/** Stops periodical CAN Bus messages that refresh values that can be read by reading()
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_ref_can::continuousReadingStop(uint8_t sensorNumber){
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStop(i);
	else{
		uint8_t data[8] = { COMMAND_REFLECTANCE_ARRAY_MEASURE_STOP };
		esp32CANBus->messageSend(idIn[sensorNumber], 1, data);
	}
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
*/
void Mrm_ref_can::decodeMessage(uint8_t data[8], uint8_t sensorNumber){
	if (sensorNumber >= nextFree)
		error("Sensor doesn't exist");
	uint8_t startIndex = 0;
	switch (data[0]) {
	case COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_1_TO_3:
		startIndex = 0;
		break;
	case COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_4_TO_6:
		startIndex = 3;
		break;
	case COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_7_TO_9:
		startIndex = 6;
		break;
	default:
		error("reflArrTest");
	}
	for (uint8_t i = 0; i <= 2; i++)
		readings[sensorNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];
}


/** Prints a frame
@param msgId - CAN Bus message id
@param dlc - data load byte count
@param data - data
@return - if true, found and printed
*/
bool Mrm_ref_can::framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]) {
	for (uint8_t sensorNumber = 0; sensorNumber < nextFree; sensorNumber++)
		if (isForMe(msgId, sensorNumber)) {
			print("%s Id: 0x%04X", nameThis[sensorNumber], msgId);
			if (dlc > 0) {
				print(", data: ");
				for (uint8_t i = 0; i < dlc; i++)
					print("0x%02X ", data[i]);
			}
			print("\n\r");
			return true;
		}
	return false;
}

/** Is the frame addressed to this device?
@param canIdOut - CAN Bus id.
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - if true, it is
*/
bool Mrm_ref_can::isForMe(uint32_t canIdOut, uint8_t sensorNumber){
	if (sensorNumber >= nextFree)
		error("Sensor doesn't exist");
	return canIdOut == idOut[sensorNumber];
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Mrm_ref_can::print(const char* fmt, ...){
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Analog readings
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::reading(uint8_t receiverNumberInSensor, uint8_t sensorNumber){
	if (sensorNumber > MAX_MRM_REF_CAN || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT)
		error("Sensor doesn't exist");
	return readings[sensorNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_ref_can::readingsPrint() {
	print("Ref. array:");
	for (uint8_t sensorNumber = 0; sensorNumber < nextFree; sensorNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_REF_CAN_SENSOR_COUNT; irNo++)
		print(" %3i", readings[sensorNumber][irNo]);
	}
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
*/
void Mrm_ref_can::devicesScan(bool verbose) {
#define REPORT_STRAY 0
	for (uint8_t i = 0; i < nextFree; i++) {
		uint8_t data[8] = { COMMAND_REPORT_ALIVE };

		esp32CANBus->messageSend(idIn[i], 1, data);

		if (verbose)
			print("%s:", nameThis[i]);

		uint32_t nowMs = millis();
		bool any = false;
		while (millis() - nowMs < 10 && !any)
			if (esp32CANBus->messageReceive()) {
				if (esp32CANBus->rx_frame->MsgID == idOut[i]) {
					if (verbose)
						print("found\n\r");
					any = true;
					aliveThis[i] = true;
				}
#if REPORT_STRAY
				else
					if (verbose)
						print("stray id: %0x02X", (String)esp32CANBus->rx_frame->MsgID);
#endif
			}
		if (!any) {
			if (verbose)
				print("no response\n\r");
			aliveThis[i] = false;
		}
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_ref_can::test(BreakCondition breakWhen)
{
	continuousReadingStart();

	uint32_t lastMs = 0;
	bool newMessage = false;

	while (breakWhen == 0 || !(*breakWhen)()) {

		for (uint8_t sensorNumber = 0; sensorNumber < nextFree; sensorNumber++){

			//blink();
			if (esp32CANBus->messageReceive() && esp32CANBus->rx_frame->MsgID == idOut[sensorNumber]) {
				uint8_t startIndex = 0;
				switch (esp32CANBus->rx_frame->data.u8[0]) {
				case COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_1_TO_3:
					startIndex = 0;
					break;
				case COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_4_TO_6:
					startIndex = 3;
					break;
				case COMMAND_REFLECTANCE_ARRAY_SENDING_SENSORS_7_TO_9:
					startIndex = 6;
					break;
				default:
					error("reflArrTest");
				}
				for (uint8_t i = 0; i <= 2; i++)
					readings[sensorNumber][startIndex + i] = (esp32CANBus->rx_frame->data.u8[2 * i + 1] << 8) | esp32CANBus->rx_frame->data.u8[2 * i + 2];
				newMessage = true;
			}
			delay(5);

			if (newMessage && millis() - lastMs > 300) {
				for (uint8_t i = 0; i < MRM_REF_CAN_SENSOR_COUNT; i++)
					print("%i ", readings[sensorNumber][i]);

				print("\n\r");
				lastMs = millis();
			}
		}
	}
	print("\n\rTest over.\n\r");

	continuousReadingStop();
}

/** Print to all serial ports, pointer to list
*/
void Mrm_ref_can::vprint(const char *fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}

#include "mrm-therm-b-can.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_therm_b_can::Mrm_therm_b_can(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) {
	esp32CANBus = esp32CANBusSingleton;
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_therm_b_can::~Mrm_therm_b_can()
{
}

/** Add a mrm-therm-b-can sensor
@param deviceName - device's name
*/
void Mrm_therm_b_can::add(char * deviceName)
{
	if (nextFree >= MAX_MRM_THERM_B_CAN)
		error("Too many mrm-therm-b-can");


	switch (nextFree) {
	case 0:
		idIn[nextFree] = CAN_ID_THERM0_IN;
		idOut[nextFree] = CAN_ID_THERM0_OUT;
		break;
	case 1:
		idIn[nextFree] = CAN_ID_THERM1_IN;
		idOut[nextFree] = CAN_ID_THERM1_OUT;
		break;
	default:
		error("Too many sensors");
	}

	if (deviceName != 0) {
		if (strlen(deviceName) > 9)
			error("Name too long");
		strcpy(nameThis[nextFree], deviceName);
	}
	nextFree++;
}

/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_therm_b_can::continuousReadingStart(uint8_t sensorNumber){
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStart(i);
	else{
		uint8_t data[8] = { COMMAND_THERMO_MEASURE_CONTINUOUS };
		esp32CANBus->messageSend(idIn[sensorNumber], 1, data);
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_therm_b_can::continuousReadingStop(uint8_t sensorNumber){
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStop(i);
	else{
		uint8_t data[8] = { COMMAND_THERMO_MEASURE_STOP };
		esp32CANBus->messageSend(idIn[sensorNumber], 1, data);
	}
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@return - true if canId for this class
*/
bool Mrm_therm_b_can::decodeMessage(uint32_t canId, uint8_t data[8]){
	for (uint8_t sensorNumber = 0; sensorNumber < nextFree; sensorNumber++)
		if (isForMe(canId, sensorNumber)){
			int16_t temp = (data[1] << 8) | data[0];
			readings[sensorNumber] = temp;
			return true;
		}
	return false;
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
*/
void Mrm_therm_b_can::devicesScan(bool verbose) {
#define REPORT_STRAY 0
	for (uint8_t i = 0; i < nextFree; i++) {
		uint8_t data[8] = { COMMAND_REPORT_ALIVE };

		esp32CANBus->messageSend(idIn[i], 1, data);

		if (verbose)
			print("%s:", nameThis[i]);

		uint32_t nowMs = millis();
		bool any = false;
		while (millis() - nowMs < 25 && !any)
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

/** Prints a frame
@param msgId - CAN Bus message id
@param dlc - data load byte count
@param data - data
@return - if true, found and printed
*/
bool Mrm_therm_b_can::framePrint(uint32_t msgId, uint8_t dlc, uint8_t data[8]) {
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
bool Mrm_therm_b_can::isForMe(uint32_t canIdOut, uint8_t sensorNumber){
	if (sensorNumber >= nextFree)
		error("Sensor doesn't exist");
	return canIdOut == idOut[sensorNumber];
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Mrm_therm_b_can::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Analog readings
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - analog value
*/
int16_t Mrm_therm_b_can::reading(uint8_t sensorNumber){
	if (sensorNumber > MAX_MRM_THERM_B_CAN)
		error("Sensor doesn't exist");
	return readings[sensorNumber];
}

/** Print all readings in a line
*/
void Mrm_therm_b_can::readingsPrint() {
	print("Therm:");
	for (uint8_t sensorNumber = 0; sensorNumber < nextFree; sensorNumber++)
		if (aliveThis[sensorNumber])
			print(" %4i deg. C", readings[sensorNumber]);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_therm_b_can::test(BreakCondition breakWhen)
{
	continuousReadingStart();

	uint32_t lastMs = 0;
	print("\n\r");
	while (breakWhen == 0 || !(*breakWhen)()) {
		if (esp32CANBus->messageReceive())
			decodeMessage(esp32CANBus->rx_frame->MsgID, esp32CANBus->rx_frame->data.u8);
		if (millis() - lastMs > 300) {
			lastMs = millis();
			readingsPrint();
			print("\n\r");
		}
	}

	print("\n\rTest over.\n\r");

	continuousReadingStop();
}

/** Print to all serial ports, pointer to list
*/
void Mrm_therm_b_can::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}

#include "mrm-8x8a.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_8x8a::Mrm_8x8a(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) {
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
	if (nextFree >= MAX_MRM_8X8A)
		error("Too many mrm-8x8a");
	idIn[nextFree] = CAN_ID_LED8x8_IN;
	idOut[nextFree] = CAN_ID_LED8x8_OUT;
	if (deviceName != 0) {
		if (strlen(deviceName) > 9)
			error("Name too long");
		strcpy(nameThis[nextFree], deviceName);
	}
	nextFree++;
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Mrm_8x8a::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
*/
void Mrm_8x8a::devicesScan(bool verbose) {
#define REPORT_STRAY 0
	for (uint8_t i = 0; i < nextFree; i++) {
		uint8_t data[8] = { COMMAND_REPORT_ALIVE };

		esp32CANBus->messageSend(idIn[i], 1, data);

		if (verbose)
			print("%s:", nameThis[i]);

		uint32_t nowMs = millis();
		bool any = false;
		while (millis() - nowMs < 100 && !any)
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
		if (!any){
			if (verbose)
				print("no response\n\r");
			aliveThis[i] = false;
		}
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_8x8a::test(BreakCondition breakWhen)
{

	print("\n\rTest over.\n\r");
}

/** Print to all serial ports, pointer to list
*/
void Mrm_8x8a::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}

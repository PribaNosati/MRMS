#include "mrm-switch.h"
#include <mrm-robot.h>

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_switch::Mrm_switch(Robot* robot, uint8_t maxDevices) : 
	SensorBoard(robot, 1, "LED8x8", maxDevices) {
	lastOn = new std::vector<bool[MRM_SWITCHES_COUNT]>(maxDevices);
	offOnAction = new std::vector<ActionBase* [MRM_SWITCHES_COUNT]>(maxDevices);
	pin = new std::vector<uint8_t[MRM_SWITCHES_COUNT]>(maxDevices);
	nextFree = 0;
}

Mrm_switch::~Mrm_switch()
{
}


ActionBase* Mrm_switch::actionCheck() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t switchNumber = 0; switchNumber < MRM_SWITCHES_COUNT; switchNumber++)
			if ((*lastOn)[deviceNumber][switchNumber] == false && read(switchNumber, deviceNumber) && (*offOnAction)[deviceNumber][switchNumber] != NULL) {
				((*lastOn)[deviceNumber][switchNumber]) = true;
				return (*offOnAction)[deviceNumber][switchNumber];
			} else if ((*lastOn)[deviceNumber][switchNumber] == true && !read(switchNumber, deviceNumber))
				((*lastOn)[deviceNumber][switchNumber]) = false;
	}
	return NULL;
}

void Mrm_switch::actionSet(ActionBase* action, uint8_t switchNumber, uint8_t deviceNumber) {
	(*offOnAction)[deviceNumber][switchNumber] = action;
}


/** Add a mrm-8x8a board
@param pin1 - ESP32 pin the first switch is connected to. Enter 0xFF if not in use.
@param pin2 - ESP32 pin the second switch is connected to. Enter 0xFF if not in use.
@param deviceName - device's name
*/
void Mrm_switch::add(uint8_t pin1, uint8_t pin2, char* deviceName)
{
	for (uint8_t i = 0; i < MRM_SWITCHES_COUNT; i++) {
		(*lastOn)[nextFree][i] = false;
		(*offOnAction)[nextFree][i] = NULL;
	}

	(*pin)[nextFree][0] = pin1;
	(*pin)[nextFree][1] = pin2;
	if (pin1 != 0xFF)
		pinMode(pin1, INPUT);
	if (pin2 != 0xFF)
		pinMode(pin2, INPUT);

	SensorBoard::add(deviceName, 0, 0);
}

/** Read switch
@param switchNumber
@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if pressed, false otherwise
*/
bool Mrm_switch::read(uint8_t switchNumber, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || switchNumber >= MRM_SWITCHES_COUNT) {
		strcpy(robotContainer->errorMessage, "Switch doesn't exist");
		return false;
	}
	return digitalRead((*pin)[deviceNumber][switchNumber]) == HIGH;
}


/**Test
*/
void Mrm_switch::test()
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			//if (alive(deviceNumber)) {
				print("Sw:");
				for (uint8_t i = 0; i < MRM_SWITCHES_COUNT; i++)
					print("%i ", read(deviceNumber, i));
			//}
		}
		lastMs = millis();
		print("\n\r");
	}
}


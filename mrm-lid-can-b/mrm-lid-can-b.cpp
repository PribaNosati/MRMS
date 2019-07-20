#include "mrm-lid-can-b.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_lid_can_b::Mrm_lid_can_b(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_lid_can_b::~Mrm_lid_can_b()
{
}

/** Add a mrm-lid-can-b sensor
@param addressIn - inbound message id
@param addressOut - outbound message id
*/
void Mrm_lid_can_b::add(uint32_t addressIn, uint32_t addressOut)
{
	if (nextFree >= MAX_MRM_LID_CAN_B)
		error("Too many Mrm_lid_can_b");
	idIn[nextFree] = addressIn;
	idOut[nextFree] = addressOut;
	esp32CANBus = new ESP32CANBus();
	nextFree++;
}

/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_lid_can_b::continuousReadingStart(uint8_t sensorNumber){
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStart(i);
	else{
		uint8_t data[8] = { COMMAND_LIDAR_MEASURE_CONTINUOUS };
		esp32CANBus->messageSend(idIn[sensorNumber], 1, data);
	}
}

/** Stops periodical CANBus messages that refresh values that can be read by reading()
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_lid_can_b::continuousReadingStop(uint8_t sensorNumber){
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			continuousReadingStop(i);
	else{
		uint8_t data[8] = { COMMAND_LIDAR_MEASURE_STOP };
		esp32CANBus->messageSend(idIn[sensorNumber], 1, data);
	}
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Mrm_lid_can_b::print(String message, bool eol) {
	if (eol) {
		Serial.println(message);
		if (serial != 0)
			serial->println(message);
	}
	else {
		Serial.print(message);
		if (serial != 0)
			serial->print(message);
	}
}

/** Analog readings
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_can_b::reading(uint8_t sensorNumber){
	if (sensorNumber > MAX_MRM_LID_CAN_B)
		error("Sensor doesn't exist");
	return readings[sensorNumber];
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_lid_can_b::test(BreakCondition breakWhen)
{
	continuousReadingStart();
	
	uint32_t lastMs = 0;
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (uint8_t sensorNumber = 0; sensorNumber < nextFree; sensorNumber++){
		//	blink();
			if (esp32CANBus->messageReceive() && esp32CANBus->rx_frame->MsgID == idOut[sensorNumber] && millis() - lastMs > 300) {//CAN_ID_VL530X){
				lastMs = millis();

				uint16_t mm = (esp32CANBus->rx_frame->data.u8[1] << 8) | esp32CANBus->rx_frame->data.u8[0];
				readings[sensorNumber] = mm;
				Serial.println((String)mm + " mm");
			}
		}
	}

	print("\n\rTest over.", true);

	continuousReadingStart();
}

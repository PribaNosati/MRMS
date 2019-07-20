#include "mrm-ref-can_esp32.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  
extern CAN_frame_t rx_frame;

/** Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_ref_can::Mrm_ref_can(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_ref_can::~Mrm_ref_can()
{
}

/** Add a mrm-ref-can sensor
@param addressIn - inbound message id
@param addressOut - outbound message id
*/
void Mrm_ref_can::add(uint32_t addressIn, uint32_t addressOut)
{
	if (nextFree >= MAX_MRM_REF_CAN)
		error("Too many Mrm_ref_can");
	idIn[nextFree] = addressIn;
	idOut[nextFree] = addressOut;
	esp32CANBus = new ESP32CANBus();
	nextFree++;
}

/** Starts periodical CANBus messages that will be refreshing values that can be read by reading()
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

/** Stops periodical CANBus messages that refresh values that can be read by reading()
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

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Mrm_ref_can::print(String message, bool eol) {
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
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - analog value
*/
uint16_t Mrm_ref_can::reading(uint8_t receiverNumberInSensor, uint8_t sensorNumber){
	if (sensorNumber > MAX_MRM_REF_CAN || receiverNumberInSensor > MRM_REF_CAN_SENSOR_COUNT)
		error("Sensor doesn't exist");
	return readings[sensorNumber][receiverNumberInSensor];
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
			if (esp32CANBus->messageReceive() && rx_frame.MsgID == idOut[sensorNumber]) {
				uint8_t startIndex = 0;
				switch (rx_frame.data.u8[0]) {
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
					readings[sensorNumber][startIndex + i] = (rx_frame.data.u8[2 * i + 1] << 8) | rx_frame.data.u8[2 * i + 2];
				newMessage = true;

			}
			delay(5);

			if (newMessage && millis() - lastMs > 300) {
				for (uint8_t i = 0; i < MRM_REF_CAN_SENSOR_COUNT; i++)
					print((String)readings[sensorNumber][i] + " ");

				print("", true);
				lastMs = millis();
			}
		}
	}
	print("\n\rTest over.", true);

	continuousReadingStart();
}

#include "mrm-node.h"
#include <ESP32CANBus.h>

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_node::Mrm_node(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) : SensorBase(esp32CANBusSingleton, 1, "Node") {
	serial = hardwareSerial;
}

Mrm_node::~Mrm_node()
{
}

/** Add a mrm-node sensor
@param deviceName - device's name
*/
void Mrm_node::add(char * deviceName)
{
	SensorBase::add(deviceName, CAN_ID_NODE0_IN, CAN_ID_NODE0_OUT, CAN_ID_NODE1_IN, CAN_ID_NODE1_OUT,
		CAN_ID_NODE2_IN, CAN_ID_NODE2_OUT, CAN_ID_NODE3_IN, CAN_ID_NODE3_OUT, CAN_ID_NODE4_IN,
		CAN_ID_NODE4_OUT, CAN_ID_NODE5_IN, CAN_ID_NODE5_OUT, CAN_ID_NODE6_IN, CAN_ID_NODE6_OUT,
		CAN_ID_NODE7_IN, CAN_ID_NODE7_OUT);

	switches[nextFree-1] = 0;
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_node::messageDecode(uint32_t canId, uint8_t data[8]) {

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
				break;
			case COMMAND_FPS_SENDING:
				fpsLast = (data[1] << 8) | data[2];
				break;
			case COMMAND_NODE_SENDING_SENSORS_1_TO_3:
				startIndex = 0;
				any = true;
				break;
			case COMMAND_NODE_SENDING_SENSORS_4_TO_6:
				startIndex = 3;
				any = true;
				break;
			case COMMAND_NODE_SENDING_SENSORS_7_TO_9:
				startIndex = 6;
				any = true;
				break;
			case COMMAND_NODE_SWITCH_ON: {
				uint8_t switchNumber = data[1] >> 1;
				if (switchNumber > 4)
					error("No switch");
				switches[switchNumber] = data[1] & 1;
			}
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				error("NodeDec");
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
uint16_t Mrm_node::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber) {
	if (deviceNumber > MAX_SENSORS_BASE || receiverNumberInSensor > MRM_NODE_SENSOR_COUNT)
		error("Device doesn't exist");
	return readings[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_node::readingsPrint() {
	print("Ref. array:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_NODE_SENSOR_COUNT; irNo++)
			print(" %3i", readings[deviceNumber][irNo]);
	}
}

/** Test servos
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_node::servoTest(BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (uint8_t deg = 0; deg <= 180; deg += 5) {
			for (uint8_t servoNumber = 0; servoNumber < 3; servoNumber++)
				servoWrite(servoNumber, deg);
			print("%i deg.\n\r", deg);
			delay(100);
		}
	}
}

/** Move servo
@servoNumber - 0 - 2
@degrees - 0 - 180 degrees
@deviceNumber - mrm-node id
*/
void Mrm_node::servoWrite(uint8_t servoNumber, uint16_t degrees, uint8_t deviceNumber) {
	if (servoNumber > 3)
		error("Servo not found");
	if (degrees != servoDegrees[servoNumber]) {
		canData[0] = COMMAND_NODE_SERVO_SET;
		canData[1] = servoNumber;
		canData[2] = degrees >> 8;
		canData[3] = degrees & 0xFF;
		servoDegrees[servoNumber] = degrees;

		esp32CANBus->messageSend(idIn[deviceNumber], 4, canData);
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_node::test(BreakCondition breakWhen)
{
	continuousReadingStart();

	uint32_t lastMs = 0;
	bool newMessage = false;

	while (breakWhen == 0 || !(*breakWhen)()) {

		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {

			//blink();
			if (esp32CANBus->messageReceive() && esp32CANBus->rx_frame->MsgID == idOut[deviceNumber]) {
				messageDecode(esp32CANBus->rx_frame->MsgID, esp32CANBus->rx_frame->data.u8);
				newMessage = true;
			}
			delay(5);

			if (newMessage && millis() - lastMs > 300) {
				for (uint8_t i = 0; i < MRM_NODE_SENSOR_COUNT; i++)
					print("%i ", readings[deviceNumber][i]);
				print("|");
				for (uint8_t i = 0; i < 5; i++)
					print("%i ", switches[i]);

				print("\n\r");
				lastMs = millis();
			}
		}
	}
	print("\n\rTest over.\n\r");

	continuousReadingStop();
}

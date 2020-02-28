#include "mrm-node.h"

extern CAN_device_t CAN_cfg;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_node::Mrm_node(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Node", maxNumberOfBoards) {
	readings = new std::vector<uint16_t[MRM_NODE_ANALOG_COUNT]>(maxNumberOfBoards);
	switches = new std::vector<bool[MRM_NODE_SWITCHES_COUNT]>(maxNumberOfBoards);
	servoDegrees = new std::vector<uint16_t[MRM_NODE_SERVO_COUNT]>(maxNumberOfBoards);
}

Mrm_node::~Mrm_node()
{
}

/** Add a mrm-node sensor
@param deviceName - device's name
*/
void Mrm_node::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_NODE0_IN;
		canOut = CAN_ID_NODE0_OUT;
		break;
	case 1:
		canIn = CAN_ID_NODE1_IN;
		canOut = CAN_ID_NODE1_OUT;
		break;
	case 2:
		canIn = CAN_ID_NODE2_IN;
		canOut = CAN_ID_NODE3_OUT;
		break;
	case 3:
		canIn = CAN_ID_NODE3_IN;
		canOut = CAN_ID_NODE4_OUT;
		break;
	case 4:
		canIn = CAN_ID_NODE4_IN;
		canOut = CAN_ID_NODE4_OUT;
		break;
	case 5:
		canIn = CAN_ID_NODE5_IN;
		canOut = CAN_ID_NODE5_OUT;
		break;
	case 6:
		canIn = CAN_ID_NODE6_IN;
		canOut = CAN_ID_NODE6_OUT;
		break;
	case 7:
		canIn = CAN_ID_NODE7_IN;
		canOut = CAN_ID_NODE7_OUT;
		break;
	default:
		strcpy(robotContainer->errorMessage, "Too many mrm-node");
		return;
	}

	for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
		(*switches)[nextFree][i] = 0;

	for (uint8_t i = 0; i < MRM_NODE_SERVO_COUNT; i++)
		(*servoDegrees)[nextFree][i] = 0xFFFF;

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
*/
bool Mrm_node::messageDecode(uint32_t canId, uint8_t data[8]) {

	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (isForMe(canId, deviceNumber)) {
			messageDecodeCommon(deviceNumber);
			bool any = false;
			uint8_t startIndex = 0;
			switch (data[0]) {
			case COMMAND_ERROR:
				errorCode = data[1];
				errorInDeviceNumber = deviceNumber;
				print("Error %i in %s.\n\r", errorCode, (*nameThis)[deviceNumber]);
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
				if (switchNumber > 4) {
					strcpy(robotContainer->errorMessage, "No mrm-switch");
					return false;
				}
				(*switches)[deviceNumber][switchNumber] = data[1] & 1;
			}
				break;
			case COMMAND_NOTIFICATION:
				break;
			case COMMAND_REPORT_ALIVE:
				break;
			default:
				print("Unknown command 0x%x\n\r", data[0]);
				errorCode = 204;
				errorInDeviceNumber = deviceNumber;
			}

			if (any)
				for (uint8_t i = 0; i <= 2; i++)
					(*readings)[deviceNumber][startIndex + i] = (data[2 * i + 1] << 8) | data[2 * i + 2];

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
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_NODE_ANALOG_COUNT) {
		strcpy(robotContainer->errorMessage, "mrm-node doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_node::readingsPrint() {
	print("Ref. array:");
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t irNo = 0; irNo < MRM_NODE_ANALOG_COUNT; irNo++)
			print(" %3i", (*readings)[deviceNumber][irNo]);
	}
}

/** Test servos
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_node::servoTest(BreakCondition breakWhen) {
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 100) {
		for (uint8_t deg = 0; deg <= 180; deg += 5) {
			for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
				if (alive(deviceNumber)) {
					for (uint8_t servoNumber = 0; servoNumber < MRM_NODE_SERVO_COUNT; servoNumber++)
						servoWrite(servoNumber, deg, deviceNumber);
				}
			}
			print("%i deg.\n\r", deg);
			delay(100);
		}
		lastMs = millis();
	}
}

/** Move servo
@servoNumber - 0 - 2
@degrees - 0 - 180 degrees
@deviceNumber - mrm-node id
*/
void Mrm_node::servoWrite(uint8_t servoNumber, uint16_t degrees, uint8_t deviceNumber) {
	if (servoNumber >= MRM_NODE_SERVO_COUNT) {
		strcpy(robotContainer->errorMessage, "Servo not found");
		return;
	}
	if (degrees != (*servoDegrees)[deviceNumber][servoNumber]) {
		canData[0] = COMMAND_NODE_SERVO_SET;
		canData[1] = servoNumber;
		canData[2] = degrees >> 8;
		canData[3] = degrees & 0xFF;
		(*servoDegrees)[deviceNumber][servoNumber] = degrees;

		robotContainer->esp32CANBus->messageSend((*idIn)[deviceNumber], 4, canData);
	}
}

/** Read digital
@param switchNumber
@deviceNumber - mrm-node id
@return
*/
bool Mrm_node::switchRead(uint8_t switchNumber, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || switchNumber >= MRM_NODE_SWITCHES_COUNT) {
		strcpy(robotContainer->errorMessage, "Switch doesn't exist");
		return false;
	}
	return (*switches)[deviceNumber][switchNumber];
}


/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_node::test(BreakCondition breakWhen)
{
	static uint32_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print("| ");
				print("An:");
				for (uint8_t i = 0; i < MRM_NODE_ANALOG_COUNT; i++)
					print("%i ", (*readings)[deviceNumber][i]);
				print("Di:");
				for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
					print("%i ", (*switches)[deviceNumber][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

#include "mrm-bldc2x50.h"

extern CAN_device_t CAN_cfg;  

/** Constructor
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_bldc2x50::Mrm_bldc2x50(ESP32CANBus *esp32CANBusSingleton, BluetoothSerial * hardwareSerial) {
	esp32CANBus = esp32CANBusSingleton;
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_bldc2x50::~Mrm_bldc2x50()
{
}

/** Add a motor attached to a mrm-bldc2x50 motor controller
@param isReversed - changes rotation direction.
@param isLeft - is on the left side
@param deviceName - device's name
*/
void Mrm_bldc2x50::add(bool isReversed, bool isLeft, char * deviceName)
{
	if (nextFree >= MAX_MRM_BLDC2x50)
		error("Too many Mrm_bldc2x50");

	switch (nextFree) {
	case 0:
		idIn[nextFree] = CAN_ID_BLDC2X50_MOTOR0_IN;
		idOut[nextFree] = CAN_ID_BLDC2X50_MOTOR0_OUT;
		break;
	case 1:
		idIn[nextFree] = CAN_ID_BLDC2X50_MOTOR1_IN;
		idOut[nextFree] = CAN_ID_BLDC2X50_MOTOR1_OUT;
		break;
	case 2:
		idIn[nextFree] = CAN_ID_BLDC2X50_MOTOR2_IN;
		idOut[nextFree] = CAN_ID_BLDC2X50_MOTOR2_OUT;
		break;
	case 3:
		idIn[nextFree] = CAN_ID_BLDC2X50_MOTOR3_IN;
		idOut[nextFree] = CAN_ID_BLDC2X50_MOTOR3_OUT;
		break;
	default:
		error("Too many motors");
	}

	if (deviceName != 0) {
		if (strlen(deviceName) > 9)
			error("Name too long");
		strcpy(nameThis[nextFree], deviceName);
	}
	reversed[nextFree] = isReversed;
	left[nextFree] = isLeft;
	nextFree++;
}

/** Start all motors
@param leftSpeed
@param right Speed
*/
void Mrm_bldc2x50::go(int8_t leftSpeed, int8_t rightSpeed) {
	for (uint8_t i = 0; i < nextFree; i++) {
		delay(3);
		if (left[i])
			setSpeed(i, leftSpeed);
		else
			setSpeed(i, rightSpeed);
	}
}

/** Returns device's name
@param motorNumber - Motor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - name
*/
String Mrm_bldc2x50::name(uint8_t motorNumber) {
	return nameThis[motorNumber];
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Mrm_bldc2x50::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Ping devices and refresh alive array
@param verbose - prints statuses
*/
void Mrm_bldc2x50::devicesScan(bool verbose) {
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

/** Motor speed
@param motorNumber - motor's number
@param speed - in range -127 to 127
*/
void Mrm_bldc2x50::setSpeed(uint8_t motorNumber, int8_t speed) {
	if (nextFree >= MAX_MRM_BLDC2x50)
		error("Motor doesn't exist");

	if (reversed[motorNumber])
		speed = -speed;

	//TxData[0] = speed + 128;
	uint8_t data = speed + 128;
	esp32CANBus->messageSend(idIn[motorNumber], 1, &data);
}


/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_bldc2x50::test(BreakCondition breakWhen)
{
	const uint16_t PAUSE_MS = 80;
	const uint16_t DISPLAY_PAUSE_MS = 300;
	const uint8_t STEP = 1;
	const uint8_t MAXIMUM_SPEED = 100; // Max. 127

	// Select motor
	int8_t selectedMotor = -2;
	uint32_t lastMs;
	while (selectedMotor < -1 || selectedMotor >= nextFree) {
		print("Enter motor number [0-4] or wait for all\n\r");
		lastMs = millis();
		selectedMotor = -1;
		while (millis() - lastMs < 3000 && selectedMotor == -1)
			if (Serial.available()) {
				selectedMotor = Serial.read() - 48;
				//UARTsListen(0, 1);
			}
		if (selectedMotor == -1)
			print("Test all\n\r");
		else if (selectedMotor >= 0 && selectedMotor < nextFree)
			print("\n\rTest motor %i\n\r", selectedMotor);
		else
			print("\n\rMotor %i doesn't exist\n\r", selectedMotor);
	}

	// Select speed
	int16_t selectedSpeed = 0;
	bool fixedSpeed = false;
	print("Enter speed [0-127] or wait for all\n\r");
	lastMs = millis();
	while (millis() - lastMs < 2000)
		if (Serial.available()) {
			selectedSpeed = selectedSpeed * 10 + (Serial.read() - 48);
			//UARTsListen(0, 1);
			fixedSpeed = true;
			lastMs = millis();
		}
	if (fixedSpeed)
		print("\n\rSpeed %i\n\r", selectedSpeed);
	else
		print("All speeds\n\r");

	bool goOn = true;
	lastMs = 0;
	while (goOn) {
		for (volatile uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {

			if (selectedMotor != -1 && motorNumber != selectedMotor)
				continue;

			if (fixedSpeed) {
				setSpeed(motorNumber, selectedSpeed);
				if ((*breakWhen)())
					goOn = false;
				delay(PAUSE_MS);
				continue;
			}

			for (int16_t i = 0; i <= MAXIMUM_SPEED && goOn; i += STEP) {
				//blink();
				if ((*breakWhen)())
					goOn = false;
				setSpeed(motorNumber, i);
				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("%i:%i\n\r", motorNumber, i);
					lastMs = millis();
				}
				delay(PAUSE_MS);
			}
			for (int16_t i = MAXIMUM_SPEED; i >= -MAXIMUM_SPEED && goOn; i -= STEP) {
				//blink();
				if ((*breakWhen)())
					goOn = false;
				setSpeed(motorNumber, i);
				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("%i:%i\n\r", motorNumber, i);
					lastMs = millis();
				}
				delay(PAUSE_MS);
			}
			for (int16_t i = -MAXIMUM_SPEED; i < 0 && goOn; i += STEP) {
				//blink();
				if ((*breakWhen)())
					goOn = false;
				setSpeed(motorNumber, i);
				if (millis() - lastMs > DISPLAY_PAUSE_MS) {
					print("%i:%i\n\r", motorNumber, i);
					lastMs = millis();
				}
				delay(PAUSE_MS);
			}
			setSpeed(motorNumber, 0);
		}
	}

	// Stop all motors
	for (uint8_t motorNumber = 0; motorNumber < nextFree; motorNumber++) {
		delay(2);
		setSpeed(motorNumber, 0);
	}
}

/** Print to all serial ports, pointer to list
*/
void Mrm_bldc2x50::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}
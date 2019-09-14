#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32CANBus.h>
#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-ref-can.h>
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>
#include <Wire.h>


// Defines

#define COMMANDS_LIMIT 50 // Increase if more commands are needed
#define DEVICE_GROUP_COUNT 10
#define LED_ERROR 15 // Pin number, hardware defined
#define LED_OK 2 // Pin number, hardware defined


// Structures

// A command can be executed (processed) after user enters its shortcut, but also act as a state machine.
struct Command {
	bool firstProcess = true;
	char shortcut[4];
	char text[20];
	void (*pointer)();
	uint8_t menuLevel;
};

struct Command commandCanSniff;
struct Command commandDoNothing;
struct Command commandFPS;
struct Command commandGoAhead;
struct Command commandIdChange;
struct Command commandLidarCalibrate;
struct Command commandMenuMain;
struct Command commandMenuPrint;
struct Command commandReflectanceArrayCalibrate;
struct Command commandReportCANBusDevices;
struct Command commandReset;
struct Command commandScanI2C;
struct Command commandStartBroadcasting;
struct Command commandStateLineFollow; // State machine example - following a line
struct Command commandStateRun; // State machine example - running
struct Command commandStateStop; // State machine example - stopped
struct Command commandTest8x8;
struct Command commandTestAll;
struct Command commandTestAny;
struct Command commandTestBluetooth;
struct Command commandTestI2C;
struct Command commandTestIMU;
struct Command commandTestLidars2m;
struct Command commandTestLidars4m;
struct Command commandTestNode;
struct Command commandTestNodeServos;
struct Command commandTestMotors;
struct Command commandTestReflectanceArray;
struct Command commandTestServo;
struct Command commandTestThermo;

struct Command* commands[COMMANDS_LIMIT];
struct Command* commandCurrent = &commandMenuPrint;
struct Command* commandPrevious = commandCurrent;

// Variables

char errorMessage[40]; // Global variable enables functions to set it although no passed as parameter

uint8_t fpsNextIndex = 0; // To count frames per second
uint32_t fpsMs[3] = { 0, 0, 0 };

uint8_t menuLevel = 1; // Submenus have bigger numbers

unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)

static bool verbose = false; // Verbose output


// Objects

BluetoothSerial SerialBT;
ESP32CANBus esp32CANBus;
Mrm_8x8a mrm_8x8a(&esp32CANBus, &SerialBT);
Mrm_bldc4x2_5 mrm_bldc4x2_5(&esp32CANBus, &SerialBT);
Mrm_bldc2x50 mrm_bldc2x50(&esp32CANBus, &SerialBT);
Mrm_imu mrm_imu;
Mrm_lid_can_b mrm_lid_can_b(&esp32CANBus, &SerialBT);
Mrm_lid_can_b2 mrm_lid_can_b2(&esp32CANBus, &SerialBT);
Mrm_mot4x10 mrm_mot4x10(&esp32CANBus, &SerialBT);
Mrm_mot4x3_6can mrm_mot4x3_6can(&esp32CANBus, &SerialBT);
Mrm_node mrm_node(&esp32CANBus, &SerialBT);
Mrm_ref_can mrm_ref_can(&esp32CANBus, &SerialBT);
Mrm_servo mrm_servo(&SerialBT);
Mrm_therm_b_can mrm_therm_b_can(&esp32CANBus, &SerialBT);
DeviceGroup* deviceGroup[DEVICE_GROUP_COUNT] = { &mrm_8x8a, &mrm_bldc4x2_5, &mrm_bldc2x50, &mrm_lid_can_b, &mrm_lid_can_b2, &mrm_mot4x10, &mrm_mot4x3_6can, &mrm_node,
	&mrm_ref_can, &mrm_therm_b_can};


// Function declarations

void menuAdd(struct Command* command, char* shortcut, char* text, void (*pointer)(), uint8_t menuLevel);
void print(const char* fmt, ...);


// Code

/** Runs once
*/
void setup() {
	Serial.begin(115200);
	SerialBT.begin("ESP32"); //Start Bluetooth. ESP32 - Bluetooth device name
	Wire.begin(); // Start I2C
	delay(50);
	print("ESP32-Arduino-CAN\r\n");
	initialize();
}

/** Runs constantly
*/
void loop() {
	blink(); // Keep-alive LED. Solder jumper must be shorted in order to work in mrm-esp32.
	commandUpdate(); // Check if a key pressed and update current command buffer.
	if (commandCurrent == NULL) // If last command finished, display menu.
		menu();
	else
		commandProcess(); // Process current command. The command will be executed while currentCommand is not NULL. Here state maching processing occurs, too.
	fps(); // Measure FPS. Less than 30 - a bad thing.
	messagesReceive(); // Receive all CAN Bus messages. This call should be included in any loop, like here.
	verbosePrint(); // Print FPS and maybe some additional data
}

void blink() {
	#define LED_ON_MS 100
	#define LED_OFF_MS 1000
		static uint32_t lastBlinkMs = 0;
		static bool isOn = false;
		if ((millis() - lastBlinkMs > LED_OFF_MS && !isOn) || (millis() - lastBlinkMs > LED_ON_MS && isOn)) {
			digitalWrite(LED_OK, !isOn);
			isOn = !isOn;
			lastBlinkMs = millis();
		}
}


void bluetoothTest() {
	uint32_t startMs = millis();
	while (!userBreak()) {
		print("Time: %i ms.\r\n", millis() - startMs);
		delay(100);
	}
	commandCurrent = NULL;
}

void broadcastingStart() {
	for (uint8_t deviceNumber = 0; deviceNumber < DEVICE_GROUP_COUNT; deviceNumber++)
		deviceGroup[deviceNumber]->continuousReadingStart();
}

void canBusSniff() {
	while (!userBreak()) {
		blink();
		bool found = false;
		if (mrm_ref_can.esp32CANBus->messageReceive()) {
			for (uint8_t deviceNumber = 0; deviceNumber < DEVICE_GROUP_COUNT; deviceNumber++)
				if (deviceGroup[deviceNumber]->framePrint(mrm_lid_can_b.esp32CANBus->rx_frame->MsgID, mrm_lid_can_b.esp32CANBus->rx_frame->FIR.B.DLC,
					mrm_lid_can_b.esp32CANBus->rx_frame->data.u8)) {
					found = true;
					break;
				}

			if (!found) {
				print("Not found Id: 0x%4X ", (String)mrm_ref_can.esp32CANBus->rx_frame->MsgID);
				if (mrm_ref_can.esp32CANBus->rx_frame->FIR.B.DLC > 0)
					print(", data: ");
				for (uint8_t i = 0; i < mrm_ref_can.esp32CANBus->rx_frame->FIR.B.DLC; i++)
					print("0x2X ", mrm_ref_can.esp32CANBus->rx_frame->data.u8[i]);
				print("\n\r");
			}
		}
	}

	commandCurrent = NULL;
}

void canIdChange() {
	uint8_t last = 0;
	for (uint8_t deviceNumber = 0; deviceNumber < DEVICE_GROUP_COUNT; deviceNumber++)
		for (uint8_t subDeviceNumber = 0; subDeviceNumber < deviceGroup[deviceNumber]->deadOrAliveCount(); subDeviceNumber++)
			if (deviceGroup[deviceNumber]->alive(subDeviceNumber)) 
				print("%i. %s\n\r", ++last, deviceGroup[deviceNumber]->name(subDeviceNumber));
	if (last != 0) {
		print("Enter [1 - %i]: ", last);
		uint32_t lastMs = millis();
		uint8_t selectedNumber = 0xFF;
		while (millis() - lastMs < 30000 && selectedNumber > last && selectedNumber != 0)
			if (Serial.available()) 
				selectedNumber = Serial.read() - 48;

		if (selectedNumber > last)
			print("timeout\n\r");
		else {
			last = 0;
			DeviceGroup * selectedDevice = NULL;
			uint8_t selectedSubDevice = 0xFF;
			for (uint8_t deviceNumber = 0; deviceNumber < DEVICE_GROUP_COUNT && selectedSubDevice == 0xFF; deviceNumber++)
				for (uint8_t subDeviceNumber = 0; subDeviceNumber < deviceGroup[deviceNumber]->deadOrAliveCount() && selectedSubDevice == 0xFF; subDeviceNumber++)
					if (deviceGroup[deviceNumber]->alive(subDeviceNumber) && ++last == selectedNumber) {
						selectedDevice = deviceGroup[deviceNumber];
						selectedSubDevice = subDeviceNumber;
					}
			print("%i. %s\n\rEnter new group id [1..%i]: ", last, selectedDevice->name(selectedSubDevice), selectedDevice->devicesMaximumNumberInAllGroups() / selectedDevice->devicesIn1Group());
			lastMs = millis();
			uint8_t newDeviceNumber = 0xFF;
			while (millis() - lastMs < 30000 && newDeviceNumber > 5 && newDeviceNumber != 0)
				if (Serial.available())
					newDeviceNumber = Serial.read() - 48;

			if (newDeviceNumber > 5 || newDeviceNumber == 0)
				print("timeout\n\r");
			else {
				print("%i\n\rChange requested.\n\r", newDeviceNumber);
				selectedDevice->idChange(newDeviceNumber - 1, selectedSubDevice);
			}
		}
	}
	commandCurrent = NULL;
}

void commandsAdd() {
	for (uint8_t i = 0; i < COMMANDS_LIMIT; i++)
		commands[i] = NULL;

	//In all or no menus
	menuAdd(&commandMenuMain, "x", "Escape", &menuMainAndIdle, 2 | 4 | 8 | 16);//2 | 4 | 8 | 16 -> in all menus
	menuAdd(&commandMenuPrint, 0, "Menu print", &menu, 0);//0 -> in no menu
	menuAdd(&commandDoNothing, 0, "Do nothing", &doNothing, 0);//UNKNOWN_ROBOT -> display for all robots

	//Main menu (1)
	menuAdd(&commandTestMotors, "mot", "Test motors", &motorTest, 1);
	menuAdd(&commandTestLidars2m, "li2", "Test lid. 2m", &lidar2mTest, 1);
	menuAdd(&commandTestLidars4m, "li4", "Test lid. 4m", &lidar4mTest, 1);
	menuAdd(&commandScanI2C, "led", "Test 8x8", &led8x8Test, 1);
	menuAdd(&commandTestI2C, "i2c", "Test I2C", &i2cTest, 1);
	menuAdd(&commandTestIMU, "imu", "Test IMU", &imuTest, 1);
	menuAdd(&commandTestBluetooth, "blt", "Test Bluetooth", &bluetoothTest, 1);
	menuAdd(&commandTestReflectanceArray, "ref", "Test refl. arr.", &reflectanceArrayTest, 1);
	menuAdd(&commandTestNode, "nod", "Test node", &nodeTest, 1);
	menuAdd(&commandTestNodeServos, "nos", "Test node serv.", &nodeServosTest, 1);
	menuAdd(&commandReportCANBusDevices, "can", "Report devices", &devicesScan, 1);
	menuAdd(&commandCanSniff, "sni", "Sniff CAN Bus", &canBusSniff, 1);
	menuAdd(&commandReset, "rst", "Reset", &reset, 1);
	menuAdd(&commandStateLineFollow, "lin", "FollowLine", &stateLineFollow, 1);
	menuAdd(&commandStateRun, "run", "Run", &stateRun, 1);
	menuAdd(&commandStateStop, "sto", "Stop", &stateStop, 1);
	menuAdd(&commandGoAhead, "ahe", "Go ahead", &goAhead, 1);
	menuAdd(&commandStartBroadcasting, "bro", "Start sensors", &broadcastingStart, 1);
	menuAdd(&commandTestAny, "any", "Any test", &testAny, 1);
	menuAdd(&commandTestAll, "all", "All tests", &testAll, 1);
	menuAdd(&commandReflectanceArrayCalibrate, "cal", "Calibrate refl.", &reflectanceArrayCalibrate, 1);
	menuAdd(&commandTestThermo, "the", "Test thermo", &thermoTest, 1);
	menuAdd(&commandTestServo, "ser", "Test servo", &servoTest, 1);
	menuAdd(&commandLidarCalibrate, "lic", "Cal. lidar", &lidarCalibrate, 1);
	menuAdd(&commandFPS, "fps", "FPS", &fpsPrint, 1);
	menuAdd(&commandIdChange, "idc", "Device's id change", &canIdChange, 1);
}

void commandProcess() {
	if (commandCurrent != NULL) {
		(*(commandCurrent->pointer))();
		if (commandCurrent != NULL)
			commandCurrent->firstProcess = false;
	}
}

void commandUpdate() {
	static uint32_t lastUserActionMs = 0;
	static uint8_t uartRxCommandIndex = 0;
	static char uartRxCommandCumulative[10];
	const uint16_t TIMEOUT_MS = 2000;

	if (Serial.available() || SerialBT.available()) {
		lastUserActionMs = millis();
		uint8_t ch;
		if (Serial.available())
			ch = Serial.read();
		else
			ch = SerialBT.read();

		if (ch != 13) //if received data different from ascii 13 (enter)
			uartRxCommandCumulative[uartRxCommandIndex++] = ch;	//add data to Rx_Buffer

		if (ch == 13 || uartRxCommandIndex >= 3 || ch == 'x') //if received data = 13
		{
			uartRxCommandCumulative[uartRxCommandIndex] = 0;
			uartRxCommandIndex = 0;

			print("Command: %s", uartRxCommandCumulative);

			uint8_t found = 0;
			for (uint8_t i = 0; i < COMMANDS_LIMIT; i++) {
				if (strcmp(commands[i]->shortcut, uartRxCommandCumulative) == 0) {
					print(" ok.\r\n");
					commandPrevious = commandCurrent;
					commandCurrent = commands[i];
					commandCurrent->firstProcess = true;
					found = 1;
					break;
				}
			}
			if (!found) {
				print(" not found.\r\n");
				uartRxCommandIndex = 0;
			}
		}
	}

	if (uartRxCommandIndex != 0 && millis() - lastUserActionMs > TIMEOUT_MS) {
		print(" Timeout.\r\n");
		uartRxCommandIndex = 0;
	}
}

void devicesScan(bool verbose) {
	for (uint8_t i = 0; i < DEVICE_GROUP_COUNT; i++)
		deviceGroup[i]->devicesScan(verbose);
}

void devicesScan() {
	devicesScan(true);

	print("End.\r\n");
	commandCurrent = NULL;
}

void doNothing() {
}

// If anything goes wrong, display the source and stop.
void error(char * message) {
	print("%s\n\r", message);
	digitalWrite(LED_ERROR, true);
	while (1);
}

void fps() {
	fpsMs[fpsNextIndex] = millis();
	if (++fpsNextIndex >= 3)
		fpsNextIndex = 0;
}

void fpsPrint() {
	for (uint8_t i = 0; i < DEVICE_GROUP_COUNT; i++){
		deviceGroup[i]->fpsRequest();
		uint32_t startMs = millis();
		while(millis() - startMs < 50)
			messagesReceive();
		deviceGroup[i]->fpsDisplay();
	}
	commandCurrent = NULL;
}

void goAhead() {
	const uint8_t speed = 50;
	if (mrm_mot4x10.alive())
		mrm_mot4x10.go(speed, speed);
	else if (mrm_mot4x3_6can.alive())
		mrm_mot4x3_6can.go(speed, speed);
	else if (mrm_bldc4x2_5.alive())
		mrm_bldc4x2_5.go(speed, speed);
	commandCurrent = NULL;
}

void i2cScan() {
	print("Scanning.\r\n");

	bool any = false;
	for (byte address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address); // Transmission tried
		byte status = Wire.endTransmission(); // Was it successful?
		if (status == 0)
		{
			print("Found at address 0x%02x\r\n", address);
			any = true;
		}
		else if (status == 4)
			print("Error at address 0x%02x\r\n", address);
	}
	if (!any) {
		print("Nothing found.\r\n\r\n");
	}

	delay(4000);
	commandCurrent = NULL;
}

void i2cTest() {

	commandCurrent = NULL;
}

void imuTest() {
	mrm_imu.test(userBreak);

	commandCurrent = NULL;
}

void initialize() {
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

	// LEDs
	pinMode(2, OUTPUT);
	digitalWrite(2, false);
	pinMode(LED_ERROR, OUTPUT); // Error LED
	digitalWrite(15, false);

	// 8x8 LED
	mrm_8x8a.add("LED8x8_1");
	mrm_8x8a.add("LED8x8_2");

	// IMU
	mrm_imu.add(true);

	// Motors mrm-mot4x10
	mrm_mot4x10.add(false, false, "Mot4x10-1");
	mrm_mot4x10.add(false, false, "Mot4x10-2");
	mrm_mot4x10.add(false, true, "Mot4x10-3");
	mrm_mot4x10.add(false, true, "Mot4x10-4");
	mrm_mot4x10.add(false, false, "Mot4x10-5");
	mrm_mot4x10.add(false, false, "Mot4x10-6");
	mrm_mot4x10.add(false, true, "Mot4x10-7");
	mrm_mot4x10.add(false, true, "Mot4x10-8");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can.add(false, false, "Mot3.6-1");
	mrm_mot4x3_6can.add(false, false, "Mot3.6-2");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-3");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-4");
	mrm_mot4x3_6can.add(false, false, "Mot3.6-5");
	mrm_mot4x3_6can.add(false, false, "Mot3.6-6");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-7");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-8");

	// Motors mrm-bldc2x50
	mrm_bldc2x50.add(false, false, "Mot2x50-1");
	mrm_bldc2x50.add(false, true, "Mot2x50-2");
	mrm_bldc2x50.add(false, true, "Mot2x50-3");
	mrm_bldc2x50.add(false, true, "Mot2x50-4");
	mrm_bldc2x50.add(false, false, "Mot2x50-5");
	mrm_bldc2x50.add(false, true, "Mot2x50-6");
	mrm_bldc2x50.add(false, true, "Mot2x50-7");
	mrm_bldc2x50.add(false, true, "Mot2x50-8");

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5.add(false, false, "Mo4x2.5-1");
	mrm_bldc4x2_5.add(false, false, "Mo4x2.5-2");
	mrm_bldc4x2_5.add(false, true, "Mo4x2.5-3");
	mrm_bldc4x2_5.add(false, true, "Mo4x2.5-4");
	mrm_bldc4x2_5.add(false, false, "Mo4x2.5-5");
	mrm_bldc4x2_5.add(false, false, "Mo4x2.5-6");
	mrm_bldc4x2_5.add(false, true, "Mo4x2.5-7");
	mrm_bldc4x2_5.add(false, true, "Mo4x2.5-8");

	// Lidars mrm-lid-can-b, VL53L0X, 2 m
	mrm_lid_can_b.add("Lidar2m-1");
	mrm_lid_can_b.add("Lidar2m-2");
	mrm_lid_can_b.add("Lidar2m-3");
	mrm_lid_can_b.add("Lidar2m-4");
	mrm_lid_can_b.add("Lidar2m-5");
	mrm_lid_can_b.add("Lidar2m-6");
	mrm_lid_can_b.add("Lidar2m-7");
	mrm_lid_can_b.add("Lidar2m-8");

	// Lidars mrm-lid-can-b2, VL53L1X, 4 m
	mrm_lid_can_b2.add("Lidar4m-1");
	mrm_lid_can_b2.add("Lidar4m-2");
	mrm_lid_can_b2.add("Lidar4m-3");
	mrm_lid_can_b2.add("Lidar4m-4");
	mrm_lid_can_b2.add("Lidar4m-5");
	mrm_lid_can_b2.add("Lidar4m-6");
	mrm_lid_can_b2.add("Lidar4m-7");
	mrm_lid_can_b2.add("Lidar4m-8");

	// CAN Bus node
	mrm_node.add("Node-1");
	mrm_node.add("Node-2");
	mrm_node.add("Node-3");
	mrm_node.add("Node-4");
	mrm_node.add("Node-5");
	mrm_node.add("Node-6");
	mrm_node.add("Node-7");
	mrm_node.add("Node-8");

	// Reflective array
	mrm_ref_can.add("RefArr-1");
	mrm_ref_can.add("RefArr-2");
	mrm_ref_can.add("RefArr-3");
	mrm_ref_can.add("RefArr-4");
	mrm_ref_can.add("RefArr-5");
	mrm_ref_can.add("RefArr-6");
	mrm_ref_can.add("RefArr-7");
	mrm_ref_can.add("RefArr-8");

	// Servo motors
	mrm_servo.add(16, "Servo", 10);

	// Thermal array
	mrm_therm_b_can.add("Thermo-1");
	mrm_therm_b_can.add("Thermo-2");
	mrm_therm_b_can.add("Thermo-3");
	mrm_therm_b_can.add("Thermo-4");
	mrm_therm_b_can.add("Thermo-5");
	mrm_therm_b_can.add("Thermo-6");
	mrm_therm_b_can.add("Thermo-7");
	mrm_therm_b_can.add("Thermo-8");

	commandsAdd();
}

void led8x8Test() {
	mrm_8x8a.test(userBreak);
	commandCurrent = NULL;
}

void lidar2mTest() {
	if (commandTestLidars2m.firstProcess)
		mrm_lid_can_b.continuousReadingStart();
	mrm_lid_can_b.test();
}

void lidar4mTest() {
	if (commandTestLidars4m.firstProcess)
		mrm_lid_can_b2.continuousReadingStart();
	mrm_lid_can_b2.test();
}

void lidarCalibrate() {
	print("Lidar calibration\n\r");

	// Select lidar 2 or 4 m
	int8_t selectedLidar = -1;
	uint32_t lastMs;
	while (selectedLidar != 2 && selectedLidar != 4) {
		print("Enter max distance [2 or 4]m or wait to abort\n\r");
		lastMs = millis();
		selectedLidar = -1;
		while (millis() - lastMs < 10000 && selectedLidar == -1)
			if (Serial.available()) {
				uint8_t ch = Serial.read() - 48;
				print("%i", ch);
				selectedLidar = ch;
			}
		if (selectedLidar == -1) {
			print("\n\rAbort");
			break;
		}
	}

	// Select lidar number
	if (selectedLidar != -1) {
		int8_t lidarNumber = -2;
		uint32_t lastMs;
		while (lidarNumber <= 0 || lidarNumber >= 5) {
			print("Enter lidar number [1 - 4] or wait to abort\n\r");
			lastMs = millis();
			lidarNumber = -1;
			while (millis() - lastMs < 10000 && lidarNumber == -1)
				if (Serial.available()) {
					uint8_t ch = Serial.read() - 48;
					print("%i", ch);
					lidarNumber = ch;
				}
			if (lidarNumber == -1) {
				print("\n\rAbort");
				break;
			}
			else {
				print("\n\rCalibrate lidar nr. %i.\n\r", lidarNumber);
				if (selectedLidar == 2)
					mrm_lid_can_b.calibration(lidarNumber - 1);
				else
					mrm_lid_can_b2.calibration(lidarNumber - 1);
			}
		}
	}

	commandCurrent = NULL;
}

void menu() {
	// Print menu
	devicesScan(false);
	print("\r\n");
	bool any = false;
	uint8_t column = 1;
	uint8_t maxColumns = 2;
	for (uint8_t i = 0; i < COMMANDS_LIMIT && commands[i] != NULL; i++) {
		if ((commands[i]->menuLevel | menuLevel) == commands[i]->menuLevel) {
			print("%-3s - %-22s%s", commands[i]->shortcut, commands[i]->text, column == maxColumns ? "\n\r" : "");
			delay(2);
			any = true;
			if (column++ == maxColumns)
				column = 1;
		}
	}
	if (!any)
		print("Menu level %i empty.\r\n", menuLevel);
	else
		if (column != 1)
			print("\r\n");

	// Display errors
	for (uint8_t deviceNumber = 0; deviceNumber < DEVICE_GROUP_COUNT; deviceNumber++)
		if (deviceGroup[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", deviceGroup[deviceNumber]->errorCodeLast(), deviceGroup[deviceNumber]->name(deviceGroup[deviceNumber]->errorWasInDeviceNumber()));

	commandCurrent = &commandDoNothing;
}

void menuAdd(struct Command* command, char* shortcut, char* text, void (*pointer)(), uint8_t menuLevel) {
	static uint8_t nextFree = 0;
	if (nextFree >= COMMANDS_LIMIT)
		error("COMMANDS_LIMIT exceeded.");
	if (shortcut != 0)
		strcpy(command->shortcut, shortcut);
	if (text != 0)
		strcpy(command->text, text);
	command->pointer = pointer;
	command->menuLevel = menuLevel;
	commands[nextFree++] = command;
}

void menuMainAndIdle() {
	stateStop();

	menuLevel = 1;
	commandCurrent = NULL;
}

void messagesReceive() {
	while (esp32CANBus.messageReceive()) {
		uint32_t id = esp32CANBus.rx_frame->MsgID;
		bool any = false;
		for (uint8_t deviceGroupNumber = 0; deviceGroupNumber < DEVICE_GROUP_COUNT; deviceGroupNumber++) {
			if (deviceGroup[deviceGroupNumber]->messageDecode(id, esp32CANBus.rx_frame->data.u8)) {
				any = true;
				break;
			}
		}

		if (!any)
			print("Unknown: 0x%X\n\r", id);
	}
}

void motorTest() {
	print("Test motors\n\r");
	if (mrm_mot4x10.aliveCount() > 0)
		mrm_mot4x10.test(userBreak, messagesReceive, blink);
	else if (mrm_mot4x3_6can.aliveCount() > 0)
		mrm_mot4x3_6can.test(userBreak, messagesReceive, blink);
	else if (mrm_bldc2x50.aliveCount() > 0)
		mrm_bldc2x50.test(userBreak, messagesReceive, blink);
	else if (mrm_bldc4x2_5.aliveCount() > 0)
		mrm_bldc4x2_5.test(userBreak, messagesReceive, blink);
	commandCurrent = NULL;
}

void nodeServosTest() {
	mrm_node.servoTest(userBreak);
	commandCurrent = NULL;
}

void nodeTest() {
	mrm_node.test(userBreak);
	commandCurrent = NULL;
}

/** Print to all serial ports, variable arguments
*/
void print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

void reflectanceArrayCalibrate() {
	mrm_ref_can.calibrate();
	commandCurrent = NULL;
}

void reflectanceArrayTest() {
	if (commandTestReflectanceArray.firstProcess) 
		mrm_ref_can.continuousReadingStart();
	mrm_ref_can.test();
}

void reset() {
}

void servoSweep() {
	// If variables are not needed in any other function, and  must be persistena, they should be declared static:
	static uint8_t servoDegrees = 90;
	static bool servoIncreasing = true;
	static uint32_t servoLastChangeMs = 0;

	if (millis() - servoLastChangeMs > 12) { //Servo cannot operate faster
		servoDegrees += (servoIncreasing ? 5 : -5);
		if (servoDegrees == 180 || servoDegrees == 0)
			servoIncreasing = !servoIncreasing;
		servoLastChangeMs = millis();
		mrm_servo.servoWrite(servoDegrees);
	}
}

void servoTest() {
	mrm_servo.test(userBreak);
	commandCurrent = NULL;
}

void stateLineFollow() {
	const int FORWARD_SPEED = 70;
	const int MAX_SPEED_DIFFERENCE = 150;

	if (commandStateLineFollow.firstProcess)
		mrm_ref_can.continuousReadingStart();

	static uint32_t lastMs = 0;
	uint16_t center = (mrm_ref_can.esp32CANBus->rx_frame->data.u8[1] << 8) | mrm_ref_can.esp32CANBus->rx_frame->data.u8[0];
	if (center == 0)
		mrm_mot4x3_6can.go(FORWARD_SPEED, FORWARD_SPEED);
	else {
		float error = (center - 600) / 400.0;

		int16_t l = FORWARD_SPEED + error * MAX_SPEED_DIFFERENCE;
		if (l < -127)
			l = -127;
		if (l > 127)
			l = 127;
		int16_t r = FORWARD_SPEED - error * MAX_SPEED_DIFFERENCE;
		if (r < -127)
			r = -127;
		if (r > 127)
			r = 127;
		mrm_mot4x3_6can.go(l, r);

		if (millis() - lastMs > 500) {
			lastMs = millis();
			print("%i\r\n", center);
		}
	}
}

void stateRun() {
	if (commandStateRun.firstProcess)
		print("Running\n\r");

	servoSweep();

	commandCurrent->firstProcess = false;
}

void stateStop() {
	if (mrm_mot4x3_6can.alive())
		mrm_mot4x3_6can.go(0, 0);
	if (mrm_mot4x10.alive())
		mrm_mot4x10.go(0, 0);
	for (uint8_t i = 0; i < 4; i++)
		if (mrm_bldc2x50.alive(i)) 
			mrm_bldc2x50.speedSet(i, 0);

	mrm_bldc4x2_5.continuousReadingStop();
	mrm_ref_can.continuousReadingStop();
	mrm_lid_can_b.continuousReadingStop();
	mrm_lid_can_b2.continuousReadingStop();
	mrm_therm_b_can.continuousReadingStop();

	commandCurrent = NULL;
}

void testAll() {
	static uint32_t lastMs = 0;
	if (commandTestAll.firstProcess) {
		mrm_lid_can_b.continuousReadingStart();
		mrm_lid_can_b2.continuousReadingStart();
		mrm_therm_b_can.continuousReadingStart();
		mrm_ref_can.continuousReadingStart();
	}
	if (millis() - lastMs > 300) {
		if (mrm_lid_can_b.aliveCount() > 0) {
			mrm_lid_can_b.readingsPrint();
			print(" ");
		}
		if (mrm_lid_can_b2.aliveCount() > 0) {
			mrm_lid_can_b2.readingsPrint();
			print(" ");
		}
		if (mrm_therm_b_can.aliveCount() > 0) {
			mrm_therm_b_can.readingsPrint();
			print(" ");
		}
		if (mrm_ref_can.aliveCount() > 0)
			mrm_ref_can.readingsPrint();
		print("\n\r");
		lastMs = millis();
	}
}

void testAny() {
	static uint32_t lastMs = 0;
	if (millis() - lastMs > 200) {
		mrm_lid_can_b.devicesScan();
		bool isAlive = mrm_lid_can_b.alive(0);
		print("%i alive: %i\n\r", millis(), isAlive);
		lastMs = millis();
	}
	//while (!userBreak()) {
	//	print("%i, %i\n\r", analogRead(36), analogRead(39));
	//	delay(100);
	//}

}

void thermoTest() {
	if (commandTestThermo.firstProcess) 
		mrm_therm_b_can.continuousReadingStart();
	mrm_therm_b_can.test();
}

bool userBreak() {
	if (/*switchOn() ||*/ Serial.available()) {
		return true;
	}
	else
		return false;
}

void verbosePrint() {
	if (verbose) {
		static uint32_t lastMs = 0;
		if (lastMs == 0 || millis() - lastMs > 5000) {
			uint8_t lastFPS = (fpsNextIndex == 0 ? 2 : fpsNextIndex - 1);
			uint8_t firstFPS = fpsNextIndex;
			float fps;
			if (fpsMs[lastFPS] == 0 || fpsMs[lastFPS] - fpsMs[firstFPS] == 0)
				fps = 0;
			else
				fps = 2 * 1000 / (float)(fpsMs[lastFPS] - fpsMs[firstFPS]);
			print("%i fps\r\n", (int)round(fps));
			lastMs = millis();
		}
	}
}

void verboseToggle() {
	verbose = !verbose;
	commandCurrent = &commandDoNothing;
}

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	SerialBT.print(buffer);
}
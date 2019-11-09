#include <Arduino.h>
#include <BluetoothSerial.h>
#include <mrm-8x8a.h>
#include <mrm-board.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-imu.h>
#include <mrm-pid.h>
#include <mrm-ir-finder2.h>
#include <mrm-ir-finder-can.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-ref-can.h>
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>
#include <Wire.h>


// Defines

#define COMMANDS_LIMIT 50 // Increase if more commands are needed
#define LED_ERROR 15 // Pin number, hardware defined
#define LED_OK 2 // Pin number, hardware defined
#define MOTOR_GROUP 2 // 0 - Soccer BLDC, 1 - Soccer BDC 2 x mrm-mot2x50, 2 - differential mrm-mot4x3.6, 3 - Soccer BDC mrm-mot4x10
#define MRM_BOARD_COUNT 12


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
struct Command commandSoccerIdle;
struct Command commandSoccerCatch;
struct Command commandSoccerPlayStart;
struct Command commandBroadcastingStart;
struct Command commandLineFollow; // State machine example - following a line
struct Command commandStop; // State machine example - stopped
struct Command commandTest8x8;
struct Command commandTestAll;
struct Command commandTestAny;
struct Command commandTestBluetooth;
struct Command commandTestI2C;
struct Command commandTestIMU;
struct Command commandTestIRFinder;
struct Command commandTestIRFinderCan;
struct Command commandTestIRFinderCanCalculated;
struct Command commandTestLidars2m;
struct Command commandTestLidars4m;
struct Command commandTestNode;
struct Command commandTestNodeServos;
struct Command commandTestMotors;
struct Command commandTestOmniWheels;
struct Command commandTestReflectanceArray;
struct Command commandTestServo;
struct Command commandTestThermo;

struct Command* commands[COMMANDS_LIMIT];
struct Command* commandCurrent = &commandMenuPrint;
struct Command* commandPrevious = commandCurrent;

// Variables

char errorMessage[60] = ""; // Global variable enables functions to set it although not passed as parameter

uint8_t fpsNextIndex = 0; // To count frames per second
uint32_t fpsMs[3] = { 0, 0, 0 };

float headingToMaintain; // Soccer

uint8_t menuLevel = 1; // Submenus have bigger numbers

unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)

static bool verbose = false; // Verbose output


// Objects

BluetoothSerial SerialBT;
ESP32CANBus esp32CANBus;
MotorGroupDifferential* motorGroupDifferential = NULL;
MotorGroupStar *motorGroupStar = NULL;
Mrm_8x8a mrm_8x8a(&esp32CANBus, &SerialBT);
Mrm_bldc4x2_5 mrm_bldc4x2_5(&esp32CANBus, &SerialBT);
Mrm_bldc2x50 mrm_bldc2x50(&esp32CANBus, &SerialBT);
Mrm_imu mrm_imu;
Mrm_ir_finder2 mrm_ir_finder2;
Mrm_ir_finder_can mrm_ir_finder_can(&esp32CANBus, &SerialBT);
Mrm_lid_can_b mrm_lid_can_b(&esp32CANBus, &SerialBT, 10);
Mrm_lid_can_b2 mrm_lid_can_b2(&esp32CANBus, &SerialBT);
Mrm_mot2x50 mrm_mot2x50(&esp32CANBus, &SerialBT);//
Mrm_mot4x10 mrm_mot4x10(&esp32CANBus, &SerialBT);
Mrm_mot4x3_6can mrm_mot4x3_6can(&esp32CANBus, &SerialBT);
Mrm_node mrm_node(&esp32CANBus, &SerialBT);
Mrm_ref_can mrm_ref_can(&esp32CANBus, &SerialBT);
Mrm_servo mrm_servo(&SerialBT);
Mrm_therm_b_can mrm_therm_b_can(&esp32CANBus, &SerialBT);
Board* deviceGroup[MRM_BOARD_COUNT] = { &mrm_8x8a, &mrm_bldc4x2_5, &mrm_bldc2x50, &mrm_ir_finder_can, &mrm_lid_can_b, &mrm_lid_can_b2, &mrm_mot2x50, &mrm_mot4x10, 
&mrm_mot4x3_6can, &mrm_node, &mrm_ref_can, &mrm_therm_b_can};
Mrm_pid pidXY(0.5, 200, 0); // PID controller, regulates motors' speeds for linear motion in the plane
Mrm_pid pidRotation(0.5, 100, 0); // PID controller, regulates rotation around z axis


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
	errors();
}

void blink() {
	const uint16_t onMs = 100; 
	const uint16_t offMs = 1000;
	uint8_t repeatOnTimes;
	static uint32_t lastBlinkMs = 0;
	static uint8_t isOn = 0;
	static uint8_t pass = 0;

	if (strcmp(errorMessage, "") == 0)
		repeatOnTimes = 1;
	else 
		repeatOnTimes = 2;

	if (pass < repeatOnTimes) {
		if (millis() - lastBlinkMs > onMs) {
			isOn = !isOn;
			if (!isOn)
				pass++;
			digitalWrite(LED_OK, isOn);
			lastBlinkMs = millis();
		}
	}
	else if (millis() - lastBlinkMs > offMs) {
		pass = 0;
		lastBlinkMs = 0;
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
	for (uint8_t deviceNumber = 0; deviceNumber < MRM_BOARD_COUNT; deviceNumber++)
		deviceGroup[deviceNumber]->continuousReadingStart();

	commandCurrent = NULL;
}

void broadcastingStop() {
	for (uint8_t deviceNumber = 0; deviceNumber < MRM_BOARD_COUNT; deviceNumber++)
		deviceGroup[deviceNumber]->continuousReadingStop();

	commandCurrent = NULL;
}

void canBusSniff() {
	while (!userBreak()) {
		blink();
		bool found = false;
		if (mrm_ref_can.esp32CANBus->messageReceive()) {
			for (uint8_t deviceNumber = 0; deviceNumber < MRM_BOARD_COUNT; deviceNumber++)
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
	for (uint8_t deviceNumber = 0; deviceNumber < MRM_BOARD_COUNT; deviceNumber++)
		for (uint8_t subDeviceNumber = 0; subDeviceNumber < deviceGroup[deviceNumber]->deadOrAliveCount(); subDeviceNumber++)
			if (deviceGroup[deviceNumber]->alive(subDeviceNumber)) 
				print("%i. %s\n\r", ++last, deviceGroup[deviceNumber]->name(subDeviceNumber));
	if (last == 0)
		print("No devices\n\r");
	else{
		print("Enter device [1 - %i]: ", last);
		uint32_t lastMs = millis();
		uint8_t selectedNumber = 0;
		bool any = false;
		while (millis() - lastMs < 30000 && !any || last > 9 && millis() - lastMs < 500 && any)
			if (Serial.available()) {
				selectedNumber = selectedNumber * 10 + (Serial.read() - 48);
				any = true;
				lastMs = millis();
			}

		if (selectedNumber > last) {
			if (any)
				print("invalid");
			else
				print("timeout\n\r");
		}
		else {
			last = 0;
			Board * selectedDevice = NULL;
			uint8_t selectedSubDevice = 0xFF;
			for (uint8_t deviceNumber = 0; deviceNumber < MRM_BOARD_COUNT && selectedSubDevice == 0xFF; deviceNumber++)
				for (uint8_t subDeviceNumber = 0; subDeviceNumber < deviceGroup[deviceNumber]->deadOrAliveCount() && selectedSubDevice == 0xFF; subDeviceNumber++)
					if (deviceGroup[deviceNumber]->alive(subDeviceNumber) && ++last == selectedNumber) {
						selectedDevice = deviceGroup[deviceNumber];
						selectedSubDevice = subDeviceNumber;
					}
			//print("%i %i\n\r", selectedDevice->devicesMaximumNumberInAllGroups(), selectedDevice->devicesIn1Group());
			uint8_t maxInput = selectedDevice->devicesMaximumNumberInAllGroups() / selectedDevice->devicesIn1Group();
			print("%i. %s\n\rEnter new board id [1..%i]: ", last, selectedDevice->name(selectedSubDevice), maxInput);
			lastMs = millis();
			uint8_t newDeviceNumber = 0;
			bool any = false;
			while ((millis() - lastMs < 30000 && !any || maxInput > 9 && millis() - lastMs < 500 && any) && newDeviceNumber <= maxInput)
				if (Serial.available()) {
					newDeviceNumber = newDeviceNumber * 10 + (Serial.read() - 48);
					any = true;
					lastMs = millis();
				}

			if (newDeviceNumber > maxInput || newDeviceNumber == 0)
				print("timeout\n\r");
			else {
				print("%i\n\rChange requested.\n\r", newDeviceNumber);
				selectedDevice->idChange(newDeviceNumber - 1, selectedSubDevice);
				delay(500); // Delay for firmware handling of devices with the same ids.
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
	menuAdd(&commandTest8x8, "led", "Test 8x8", &led8x8Test, 1);
	menuAdd(&commandTestI2C, "i2c", "Test I2C", &i2cTest, 1);
	menuAdd(&commandTestIMU, "imu", "Test IMU", &imuTest, 1);
	menuAdd(&commandTestIRFinder, "irf", "Test ball analog", &irFinderTest, 1);
	menuAdd(&commandTestIRFinderCan, "irs", "Test ball CAN single", &irFinderTestCan, 1);
	menuAdd(&commandTestIRFinderCanCalculated, "irc", "Test ball CAN calcul", &irFinderTestCanCalculated, 1);
	menuAdd(&commandTestBluetooth, "blt", "Test Bluetooth", &bluetoothTest, 1);
	menuAdd(&commandTestReflectanceArray, "ref", "Test refl. arr.", &reflectanceArrayTest, 1);
	menuAdd(&commandTestNode, "nod", "Test node", &nodeTest, 1);
	menuAdd(&commandTestNodeServos, "nos", "Test node serv.", &nodeServosTest, 1);
	menuAdd(&commandReportCANBusDevices, "can", "Report devices", &devicesScan, 1);
	menuAdd(&commandCanSniff, "sni", "Sniff CAN Bus", &canBusSniff, 1);
	menuAdd(&commandReset, "rst", "Reset", &reset, 1);
	menuAdd(&commandLineFollow, "lin", "FollowLine", &lineFollow, 1);
	menuAdd(&commandStop, "sto", "Stop", &commandStopAll, 1);
	menuAdd(&commandGoAhead, "ahe", "Go ahead", &goAhead, 1);
	menuAdd(&commandBroadcastingStart, "bro", "Start sensors", &broadcastingStart, 1);
	menuAdd(&commandTestAny, "any", "Any test", &testAny, 1);
	menuAdd(&commandTestAll, "all", "All tests", &testAll, 1);
	menuAdd(&commandTestOmniWheels, "omn", "Test omni wheels", &testOmniWheels, 1);
	menuAdd(&commandReflectanceArrayCalibrate, "cal", "Calibrate refl.", &reflectanceArrayCalibrate, 1);
	menuAdd(&commandTestThermo, "the", "Test thermo", &thermoTest, 1);
	menuAdd(&commandTestServo, "ser", "Test servo", &servoTest, 1);
	menuAdd(&commandLidarCalibrate, "lic", "Cal. lidar", &lidarCalibrate, 1);
	menuAdd(&commandFPS, "fps", "FPS", &fpsPrint, 1);
	menuAdd(&commandIdChange, "idc", "Device's id change", &canIdChange, 1);
	menuAdd(&commandSoccerPlayStart, "soc", "Soccer play", &soccerPlayStart, 1);
}

void commandProcess() {
	if (commandCurrent != NULL) {
		(*(commandCurrent->pointer))();
		if (commandCurrent != NULL)
			commandCurrent->firstProcess = false;
	}
}

void commandSet(struct Command *newCommand) {
	commandPrevious = commandCurrent;
	commandCurrent = newCommand;
	commandCurrent->firstProcess = true;
}

void commandStopAll() {
	broadcastingStop();
	if (motorGroupDifferential != NULL)
		motorGroupDifferential->stop();
	if (motorGroupStar != NULL)
		motorGroupStar->stop();
	commandCurrent = NULL;
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
					commandSet(commands[i]);
					//commandPrevious = commandCurrent;
					//commandCurrent = commands[i];
					//commandCurrent->firstProcess = true;
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
	broadcastingStop();
	for (uint8_t i = 0; i < MRM_BOARD_COUNT; i++) 
		deviceGroup[i]->devicesScan(verbose);
}

void devicesScan() {
	devicesScan(true);

	print("End.\r\n");
	commandCurrent = NULL;
}

void doNothing() {
}

void errors() {
	static uint32_t lastDisplayMs = 0;
	if (strcmp(errorMessage, "") != 0) {
		if (millis() - lastDisplayMs > 10000 || lastDisplayMs == 0) {
			print("ERROR! %s\n\r", errorMessage);
			commandStopAll(); // Stop all motors
		}
	}
}

void fps() {
	fpsMs[fpsNextIndex] = millis();
	if (++fpsNextIndex >= 3)
		fpsNextIndex = 0;
}

void fpsPrint() {
	for (uint8_t i = 0; i < MRM_BOARD_COUNT; i++){
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
	if (motorGroupDifferential != NULL)
		motorGroupDifferential->go(30, 30);
	if (motorGroupStar != NULL)
		motorGroupStar->go(speed);
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

void irFinderTest() {
	mrm_ir_finder2.test();
}

void irFinderTestCan() {
	if (commandTestIRFinderCan.firstProcess)
		mrm_ir_finder_can.continuousReadingStart();
	mrm_ir_finder_can.test();
}

void irFinderTestCanCalculated() {
	if (commandTestIRFinderCanCalculated.firstProcess)
		mrm_ir_finder_can.continuousReadingCalculatedDataStart();
	mrm_ir_finder_can.testCalculated();
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

	// Motor groups
#if MOTOR_GROUP == 0
	motorGroupStar = new MotorGroupStar(&mrm_bldc2x50, 2, &mrm_bldc2x50, 3, &mrm_bldc2x50, 0, &mrm_bldc2x50, 1);
#elif MOTOR_GROUP == 1
	motorGroupStar = new MotorGroupStar(&mrm_mot2x50, 0, &mrm_mot2x50, 1, &mrm_mot2x50, 2, &mrm_mot2x50, 3);
#elif MOTOR_GROUP == 2
	motorGroupDifferential = new MotorGroupDifferential(&mrm_mot4x3_6can, 0, &mrm_mot4x3_6can, 2, &mrm_mot4x3_6can, 1, &mrm_mot4x3_6can, 3);
#else
	motorGroupStar = new MotorGroupStar(&mrm_mot4x10, 2, &mrm_mot4x10, 3, &mrm_mot4x10, 0, &mrm_mot4x10, 1);
#endif

	// 8x8 LED
	mrm_8x8a.add("LED8x8_1");

	// Motors mrm-bldc2x50
	mrm_bldc2x50.add(false, "Mot2x50-1");
	mrm_bldc2x50.add(false, "Mot2x50-2");
	mrm_bldc2x50.add(false, "Mot2x50-3");
	mrm_bldc2x50.add(false, "Mot2x50-4");

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5.add(false, "Mo4x2.5-1");
	mrm_bldc4x2_5.add(false, "Mo4x2.5-2");
	mrm_bldc4x2_5.add(false, "Mo4x2.5-3");
	mrm_bldc4x2_5.add(false, "Mo4x2.5-4");

	// IMU
	mrm_imu.add(true);

	// mrm-ir-finder2
	mrm_ir_finder2.add(34, 33);

	// mrm-ir-finder-can
	mrm_ir_finder_can.add("IRFind-1");

	// Motors mrm-mot2x50
	mrm_mot2x50.add(false, "Mot2x50-1");
	mrm_mot2x50.add(false, "Mot2x50-2");
	mrm_mot2x50.add(false, "Mot2x50-3");
	mrm_mot2x50.add(false, "Mot2x50-4");

	// Motors mrm-mot4x10
	mrm_mot4x10.add(true, "Mot4x10-1");
	mrm_mot4x10.add(true, "Mot4x10-2");
	mrm_mot4x10.add(true, "Mot4x10-3");
	mrm_mot4x10.add(true, "Mot4x10-4");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can.add(false, "Mot3.6-1");
	mrm_mot4x3_6can.add(false, "Mot3.6-2");
	mrm_mot4x3_6can.add(false, "Mot3.6-3");
	mrm_mot4x3_6can.add(false, "Mot3.6-4");

	// Lidars mrm-lid-can-b, VL53L0X, 2 m
	mrm_lid_can_b.add("Lidar2m-1");
	mrm_lid_can_b.add("Lidar2m-2");
	mrm_lid_can_b.add("Lidar2m-3");
	mrm_lid_can_b.add("Lidar2m-4");
	mrm_lid_can_b.add("Lidar2m-5");
	mrm_lid_can_b.add("Lidar2m-6");
	mrm_lid_can_b.add("Lidar2m-7");
	mrm_lid_can_b.add("Lidar2m-8");
	mrm_lid_can_b.add("Lidar2m-9");
	mrm_lid_can_b.add("Lidar2m10");

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

	// Reflective array
	mrm_ref_can.add("RefArr-1");
	mrm_ref_can.add("RefArr-2");
	mrm_ref_can.add("RefArr-3");
	mrm_ref_can.add("RefArr-4");

	// Servo motors
	mrm_servo.add(16, "Servo", 10);

	// Thermal array
	mrm_therm_b_can.add("Thermo-1");
	mrm_therm_b_can.add("Thermo-2");
	mrm_therm_b_can.add("Thermo-3");
	mrm_therm_b_can.add("Thermo-4");

	commandsAdd();
}

void led8x8Test() {
	mrm_8x8a.test(userBreak);
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
		print("Enter max distance [2 or 4]m or wait to abort ");
		lastMs = millis();
		selectedLidar = -1;
		while (millis() - lastMs < 10000 && selectedLidar == -1)
			if (Serial.available()) {
				uint8_t ch = Serial.read() - 48;
				print("%i\n\r", ch);
				selectedLidar = ch;
			}
		if (selectedLidar == -1) {
			print("- abort\n\r");
			break;
		}
	}

	// Select lidar number
	if (selectedLidar != -1) {
		int8_t lidarNumber = -2;
		uint32_t lastMs;
		while (lidarNumber <= 0 || lidarNumber >= 9) {
			print("Enter lidar number [1 - 8] or wait to abort ");
			lastMs = millis();
			lidarNumber = -1;
			while (millis() - lastMs < 10000 && lidarNumber == -1)
				if (Serial.available()) {
					uint8_t ch = Serial.read() - 48;
					print("%i\n\r", ch);
					lidarNumber = ch;
				}
			if (lidarNumber == -1) {
				print("- abort\n\r");
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

void lineFollow() {
	const int FORWARD_SPEED = 70;
	const int MAX_SPEED_DIFFERENCE = 150;

	if (commandLineFollow.firstProcess)
		mrm_ref_can.continuousReadingStart();

	uint16_t left = mrm_ref_can.reading(4);
	uint16_t right = mrm_ref_can.reading(5);

	if (left > 600 && right < 600)
		motorGroupDifferential->go(20, 50);
	else if (left < 600 && right > 600)
		motorGroupDifferential->go(50, 20);
	else if (left < 600 && right < 600)
		motorGroupDifferential->go(0, 0);
	else
		motorGroupDifferential->go(40, 40);

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
	for (uint8_t deviceNumber = 0; deviceNumber < MRM_BOARD_COUNT; deviceNumber++)
		if (deviceGroup[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", deviceGroup[deviceNumber]->errorCodeLast(), deviceGroup[deviceNumber]->name(deviceGroup[deviceNumber]->errorWasInDeviceNumber()));

	commandCurrent = &commandDoNothing;
}

void menuAdd(struct Command* command, char* shortcut, char* text, void (*pointer)(), uint8_t menuLevel) {
	static uint8_t nextFree = 0;
	if (nextFree >= COMMANDS_LIMIT) {
		strcpy(errorMessage, "COMMANDS_LIMIT exceeded.");
		return;
	}
	if (shortcut != 0)
		strcpy(command->shortcut, shortcut);
	if (text != 0)
		strcpy(command->text, text);
	command->pointer = pointer;
	command->menuLevel = menuLevel;
	commands[nextFree++] = command;
}

void menuMainAndIdle() {
	commandStopAll();

	menuLevel = 1;
	commandCurrent = NULL;
}

void messagesReceive() {
	while (esp32CANBus.messageReceive()) {
		uint32_t id = esp32CANBus.rx_frame->MsgID;
		bool any = false;
		for (uint8_t deviceGroupNumber = 0; deviceGroupNumber < MRM_BOARD_COUNT; deviceGroupNumber++) {
			if (deviceGroup[deviceGroupNumber]->messageDecode(id, esp32CANBus.rx_frame->data.u8)) {
				any = true;
				break;
			}
		}

#define REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN false
#if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
		if (!any)
			print("Address device unknown: 0x%X\n\r", id);
#endif
	}
}

void motorTest() {
	print("Test motors\n\r");
	if (mrm_mot2x50.aliveCount() > 0)
		mrm_mot2x50.test(userBreak, messagesReceive, blink);
	else if (mrm_mot4x10.aliveCount() > 0)
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
}

void nodeTest() {
	if (commandTestNode.firstProcess)
		mrm_node.continuousReadingStart();
	mrm_node.test();
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

///** A rotation needed to maintain the requested heading
//@return - Rotation
//*/
//float rotationToMaintainHeading(float headingToMaintain) {
//	float rotation = 0;// map(normalized(headingToMaintain - mrm_imu.heading()), -180, 180, -200, 200);
//	return rotation;
//}

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

void soccerCatch() {
	if (mrm_ir_finder2.anyIRSource()) {

	}
	else
		commandSet(&commandSoccerIdle);
}

void soccerIdle() {
	static float headingToMaintain;
	if (commandSoccerIdle.firstProcess)
		headingToMaintain = mrm_imu.heading();
	if (mrm_ir_finder2.anyIRSource() && false)
		commandSet(&commandSoccerCatch);
	else {
		//print("\n\rError: %i = %i - 300\n\r", (int)(300 - mrm_lid_can_b2.reading(3)), mrm_lid_can_b2.reading(3));
		float errorL = 900 - mrm_lid_can_b2.reading(3);
		float errorR = mrm_lid_can_b2.reading(1) - 900;
		motorGroupStar->goToEliminateErrors(errorL > errorR ? errorL : errorR, 150 - mrm_lid_can_b2.reading(2), headingToMaintain - mrm_imu.heading(), &pidXY, &pidRotation, true);
		//delay(500);
	}
}

void soccerPlayStart() {
	if (motorGroupStar == NULL) {
		print("Define motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu.heading();
	mrm_lid_can_b2.continuousReadingStart();
	mrm_ref_can.continuousReadingStart();
	mrm_8x8a.continuousReadingStart();
	commandSet(&commandSoccerIdle);
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
	print("%i %i %i %i %i %i %3\n\r", analogRead(2), analogRead(13), analogRead(15), analogRead(25), analogRead(26), analogRead(33), analogRead(34));
	delay(300);
	//if (mrm_ir_finder2.anyIRSource())
	//	motorGroupStar->go(mrm_ir_finder2.irSource().angle, 15);
	//else
	//	motorGroupStar->stop();
	//if (commandTestAny.firstProcess) 
	//	mrm_lid_can_b.continuousReadingStart();
	//print("%i\n\r", mrm_lid_can_b.reading(9));
	//	mrm_servo.servoWrite(90);
	//}
	//if(mrm_lid_can_b.reading(0)<100 || mrm_lid_can_b.reading(1)<100)
	//	motorGroupDifferential->go(80, 0);
	//else
	//	motorGroupDifferential->go(0, 80);
}

void testOmniWheels() {
	static uint8_t nextMove;
	static uint32_t lastMs;
	if (commandTestOmniWheels.firstProcess) {
		if (motorGroupDifferential == NULL) {
			print("Differential motor group needed.");
			commandCurrent = NULL;
			return;
		}
		nextMove = 0;
		lastMs = millis();
	}
	switch (nextMove) {
	case 0:
		if (millis() - lastMs > 2000) {
			lastMs = millis();
			nextMove = 1;
		}
		else
			motorGroupDifferential->go(20, 20, 70);
		break;
	case 1:
		if (millis() - lastMs > 15000) {
			lastMs = millis();
			nextMove = 2;
		}
		else {
			float angle = (millis() - lastMs) / 1500.0;
			int8_t x =  cos(angle) * 50;
			int8_t y =  sin(angle) * 50;
			motorGroupDifferential->go(y, y, x);
			//print("%i %i %i\n\r", (int)(angle*100), (int)(x * 100), (int)(y * 100));
			//delay(100);
		}
		break;
	case 2:
		if (millis() - lastMs > 2000) {
			lastMs = millis();
			nextMove = 0;
		}
		else
			motorGroupDifferential->go(-20, -20, -70);
		break;
	}
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
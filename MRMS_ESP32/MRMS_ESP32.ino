#include <Arduino.h>
#include <BluetoothSerial.h>
#include <esp32-hal-ledc.h>
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
#include <mrm-therm-b-can.h>
#include <Wire.h>


// Defines

#define COMMANDS_LIMIT 50 // Increase if more commands are needed
#define LED_ERROR 15 // Pin number, hardware defined
#define LED_OK 2 // Pin number, hardware defined


// Structures

struct Command {
	char shortcut[4];
	char text[20];
	void (*pointer)();
	uint8_t menuLevel;
};

struct Command commandCanSniff;
struct Command commandDoNothing;
struct Command commandGoAhead;
struct Command commandFollowLine;
struct Command commandMenuMain;
struct Command commandMenuPrint;
struct Command commandReflectanceArrayCalibrate;
struct Command commandReportCANBusDevices;
struct Command commandReset;
struct Command commandScanI2C;
struct Command commandStartBroadcasting;
struct Command commandStop;
struct Command commandTestAny;
struct Command commandTestBluetooth;
struct Command commandTestI2C;
struct Command commandTestIMU;
struct Command commandTestLidars;
struct Command commandTestMotors;
struct Command commandTestReflectanceArray;
struct Command commandTestServo;
struct Command commandTestThermo;

struct Command* commands[COMMANDS_LIMIT];
struct Command* currentCommand = &commandMenuPrint;


// Variables

char errorMessage[40]; // Global variable enables functions to set it although no passed as parameter

uint8_t fpsNextIndex = 0; // To count frames per second
uint32_t fpsMs[3] = { 0, 0, 0 };

uint8_t menuLevel = 1; // Submenus have bigger numbers

static bool verbose = false; // Verbose output

unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)


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
Mrm_therm_b_can mrm_therm_b_can(&esp32CANBus, &SerialBT);


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
	delay(100);

	print("ESP32-Arduino-CAN\r\n");

	hardwareInitialize();
	softwareInitialize();
}

/** Runs constantly
*/
void loop() {

	blink();
	processLocalCommand();
	if (currentCommand == NULL)
		menu();
	else
		(*(currentCommand->pointer))();
	processState();
	processAsync();
	fps();
	receiveMessages();
	verbosePrint();
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
	currentCommand = NULL;
}

void canBusDevicesScan(bool verbose) {
	mrm_8x8a.devicesScan(verbose);
	mrm_bldc2x50.devicesScan(verbose);
	mrm_bldc4x2_5.devicesScan(verbose);
	mrm_lid_can_b.devicesScan(verbose);
	mrm_lid_can_b2.devicesScan(verbose);
	mrm_mot4x10.devicesScan(verbose);
	mrm_mot4x3_6can.devicesScan(verbose);
	mrm_node.devicesScan(verbose);
	mrm_ref_can.devicesScan(verbose);
	mrm_therm_b_can.devicesScan(verbose);
}

void canBusDevicesScanVerbose() {
	canBusDevicesScan(true);

	print("End.\r\n");
	currentCommand = NULL;
}

void canBusSniff() {
	while (!userBreak()) {
		blink();
		bool found = false;
		if (mrm_ref_can.esp32CANBus->messageReceive()) {
			if (mrm_lid_can_b.framePrint(mrm_lid_can_b.esp32CANBus->rx_frame->MsgID, mrm_lid_can_b.esp32CANBus->rx_frame->FIR.B.DLC,
				mrm_lid_can_b.esp32CANBus->rx_frame->data.u8))
				found = true;
			if (mrm_ref_can.framePrint(mrm_ref_can.esp32CANBus->rx_frame->MsgID, mrm_ref_can.esp32CANBus->rx_frame->FIR.B.DLC,
				mrm_ref_can.esp32CANBus->rx_frame->data.u8))
				found = true;
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
	currentCommand = NULL;
}

void doNothing();

void commandsAdd() {
	for (uint8_t i = 0; i < COMMANDS_LIMIT; i++)
		commands[i] = NULL;

	//In all or no menus
	menuAdd(&commandMenuMain, "x", "Escape", &menuMainAndIdle, 2 | 4 | 8 | 16);//2 | 4 | 8 | 16 -> in all menus
	menuAdd(&commandMenuPrint, 0, "Menu print", &menu, 0);//0 -> in no menu
	menuAdd(&commandDoNothing, 0, "Do nothing", &doNothing, 0);//UNKNOWN_ROBOT -> display for all robots

	//Main menu (1)
	menuAdd(&commandTestMotors, "mot", "Test motors", &motorTest, 1);
	menuAdd(&commandTestLidars, "lid", "Test lidars", &lidarTest, 1);
	menuAdd(&commandScanI2C, "sca", "Scan I2C", &i2cScan, 1);
	menuAdd(&commandTestI2C, "i2c", "Test I2C", &i2cTest, 1);
	menuAdd(&commandTestIMU, "imu", "Test IMU", &imuTest, 1);
	menuAdd(&commandTestBluetooth, "blt", "Test Bluetooth", &bluetoothTest, 1);
	menuAdd(&commandTestReflectanceArray, "ref", "Test refl. arr.", &reflectanceArrayTest, 1);
	menuAdd(&commandReportCANBusDevices, "can", "Report devices", &canBusDevicesScanVerbose, 1);
	menuAdd(&commandCanSniff, "sni", "Sniff CAN Bus", &canBusSniff, 1);
	menuAdd(&commandReset, "rst", "Reset", &reset, 1);
	menuAdd(&commandFollowLine, "lin", "FollowLine", &lineFollow, 1);
	menuAdd(&commandGoAhead, "ahe", "Go ahead", &goAhead, 1);
	menuAdd(&commandStartBroadcasting, "bro", "Start sensors", &startBroadcasting, 1);
	menuAdd(&commandStop, "sto", "Stop", &stop, 1);
	menuAdd(&commandTestAny, "any", "Any test", &testAny, 1);
	menuAdd(&commandReflectanceArrayCalibrate, "cal", "Calibrate refl.", &reflectanceArrayCalibrate, 1);
	menuAdd(&commandTestThermo, "the", "Test thermo", &testThermo, 1);
	menuAdd(&commandTestServo, "ser", "Test servo", &testServo, 1);
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

void goAhead() {
	if (mrm_mot4x10.alive())
		mrm_mot4x10.go(80, 80);
	else if (mrm_mot4x3_6can.alive())
		mrm_mot4x3_6can.go(80, 80);
	currentCommand = NULL;
}

void hardwareInitialize() {
	// LEDs
	pinMode(2, OUTPUT);
	digitalWrite(2, false);
	pinMode(LED_ERROR, OUTPUT); // Error LED
	digitalWrite(15, false);

	// Servo: 20 ms period, duty 1 - 2 ms. 1.5 ms - neutral position.
#define CHANNEL 1
#define FREQUENCY_HZ 50
#define TIMER_WIDTH 16
	ledcSetup(CHANNEL, FREQUENCY_HZ, TIMER_WIDTH);
	ledcAttachPin(16, CHANNEL); // GPIO 16 assigned to channel 1
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
	currentCommand = NULL;
}

void i2cTest() {

	currentCommand = NULL;
}

void imuTest() {
	mrm_imu.test(userBreak);

	currentCommand = NULL;
}

void lidarTest() {
	if (mrm_lid_can_b.alive())
		mrm_lid_can_b.test(userBreak);
	if (mrm_lid_can_b2.alive())
		mrm_lid_can_b2.test(userBreak);

	currentCommand = NULL;
}

void lineFollow() {
	const int FORWARD_SPEED = 70;
	const int MAX_SPEED_DIFFERENCE = 150;

	mrm_ref_can.continuousReadingStart();
	uint32_t lastMs = 0;
	while (!userBreak()) {
		blink();
		volatile uint16_t center = (mrm_ref_can.esp32CANBus->rx_frame->data.u8[1] << 8) | mrm_ref_can.esp32CANBus->rx_frame->data.u8[0];
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

	mrm_ref_can.continuousReadingStop();

	mrm_mot4x3_6can.go(0, 0);

	currentCommand = NULL;
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


void menu() {
	canBusDevicesScan(false);
	//delay(1);
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
	currentCommand = &commandDoNothing;
}

void menuMainAndIdle() {
	//	MotorsGo(0, 0, 0, false, 1, false);
	//stateSet(IDLE);

	menuLevel = 1;
	menu();
}

void motorTest() {
	print("Test motors\n\r");
	if (mrm_mot4x10.alive())
		mrm_mot4x10.test(userBreak);
	else if (mrm_mot4x3_6can.alive())
		mrm_mot4x3_6can.test(userBreak);
	else if (mrm_bldc2x50.alive())
		mrm_bldc2x50.test(userBreak);
	else if (mrm_bldc4x2_5.alive())
		mrm_bldc4x2_5.test(userBreak);
	//mrm_bldc2x50.setSpeed(0, -15);
	//delay(1);
	//mrm_bldc2x50.setSpeed(1, 8);
	currentCommand = NULL;
}

/** Print to all serial ports, variable arguments
*/
void print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Print to all serial ports, pointer to list
*/
void vprint(const char *fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	SerialBT.print(buffer);
}

void processAsync() {

}

void processLocalCommand() {
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

		if (ch == 13 || uartRxCommandIndex >= 3 || ch == 'x')			//if received data = 13
		{
			uartRxCommandCumulative[uartRxCommandIndex] = 0;
			uartRxCommandIndex = 0;

			print(" Command: %s", uartRxCommandCumulative);

			uint8_t found = 0;
			for (uint8_t i = 0; i < COMMANDS_LIMIT; i++) {
				if (strcmp(commands[i]->shortcut, uartRxCommandCumulative) == 0) {
					print(" ok.\r\n");
					currentCommand = commands[i];
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

void processState() {
	//switch (state) {

	//case IDLE: // -> RUN_MARKER_SEARCH
	//	break;

	//default:
	//	break;
	//}

	//if (userBreak() && state != IDLE) {
	//	stateSet(IDLE);
	//}
}

void receiveMessages() {
	while (esp32CANBus.messageReceive()) {
		uint32_t id = esp32CANBus.rx_frame->MsgID;
		if (mrm_ref_can.isForMe(id))
			mrm_ref_can.decodeMessage(esp32CANBus.rx_frame->data.u8);
		else if (mrm_lid_can_b.decodeMessage(id, esp32CANBus.rx_frame->data.u8))
			;
		else if (mrm_lid_can_b2.decodeMessage(id, esp32CANBus.rx_frame->data.u8))
			;
		else if (mrm_therm_b_can.decodeMessage(id, esp32CANBus.rx_frame->data.u8))
			;
		else
			print("Unknown: %i\n\r", (int)id);
	}
}

void reflectanceArrayCalibrate() {
	mrm_ref_can.calibrate();
	currentCommand = NULL;
}

void reflectanceArrayTest() {
	mrm_ref_can.test(userBreak);
	currentCommand = NULL;
}

void reset() {
}

void servoWrite(uint16_t degrees) {
	degrees = constrain(degrees, 0, 180);
#define PERIOD 65535
	ledcWrite(1, map(degrees, 0, 180, PERIOD / 20, PERIOD / 10));
}

void softwareInitialize() {
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

	// 8x8 LED
	mrm_8x8a.add("LED8x8");

	// IMU
	mrm_imu.add(true);

	// Motors mrm-mot4x3.6can
	mrm_mot4x10.add(false, true, "Mot4x10-1");
	mrm_mot4x10.add(false, true, "Mot4x10-2");
	mrm_mot4x10.add(false, true, "Mot4x10-3");
	mrm_mot4x10.add(false, true, "Mot4x10-4");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can.add(false, true, "Mot3.6-1");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-2");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-3");
	mrm_mot4x3_6can.add(false, true, "Mot3.6-4");

	// Motors mrm-bldc2x50
	mrm_bldc2x50.add(false, true, "Mot2x50-1");
	mrm_bldc2x50.add(false, true, "Mot2x50-2");

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5.add(false, true, "Mot2.5-1");
	mrm_bldc4x2_5.add(false, true, "Mot2.5-2");
	mrm_bldc4x2_5.add(false, true, "Mot2.5-3");
	mrm_bldc4x2_5.add(false, true, "Mot2.5-4");

	// Lidars VL53L0X
	mrm_lid_can_b.add("Lidar2m1");
	mrm_lid_can_b.add("Lidar2m2");
	mrm_lid_can_b.add("Lidar2m3");
	mrm_lid_can_b.add("Lidar2m4");
	mrm_lid_can_b.add("Lidar2m5");

	// Lidars VL53L1X
	mrm_lid_can_b2.add("Lidar4m1");
	mrm_lid_can_b2.add("Lidar4m2");
	mrm_lid_can_b2.add("Lidar4m3");
	mrm_lid_can_b2.add("Lidar4m4");
	mrm_lid_can_b2.add("Lidar4m5");

	// CAN Bus node
	mrm_node.add("Node1");

	// Reflective array
	mrm_ref_can.add("Ref. arr.");

	// Thermal array
	mrm_therm_b_can.add("Thermo");

	commandsAdd();
}

void startBroadcasting() {
	mrm_ref_can.continuousReadingStart();
	mrm_lid_can_b.continuousReadingStart();
	mrm_lid_can_b2.continuousReadingStart();
	mrm_therm_b_can.continuousReadingStart();
}

void stop() {
	if (mrm_mot4x3_6can.alive())
		mrm_mot4x3_6can.go(0, 0);
	if (mrm_mot4x10.alive())
		mrm_mot4x10.go(0, 0);

	mrm_ref_can.continuousReadingStop();
	mrm_lid_can_b.continuousReadingStop();
	mrm_lid_can_b2.continuousReadingStop();
	mrm_therm_b_can.continuousReadingStop();

	currentCommand = NULL;
}

void testAny() {

	startBroadcasting();

	const uint16_t PAUSE_MS = 0;

	bool slower = true;
	uint32_t lastMs = 0;
	while (!userBreak()) {
		receiveMessages();
		if (millis() - lastMs > 500) {
			mrm_lid_can_b.readingsPrint();
			print(" ");
			mrm_ref_can.readingsPrint();
			print(" ");
			mrm_therm_b_can.readingsPrint();
			print("\n\r");
			lastMs = millis();
		}
		//delay(PAUSE_MS);
		//motorSetSpeed(0, slower ? 20 : 75);
		//if (millis() - lastChangeMs > 1000) {
		//	slower = !slower;
		//	lastChangeMs = millis();
		//}
	}
	mrm_mot4x3_6can.go(0, 0);
	currentCommand = NULL;
}

void testServo() {
	while (!userBreak()) {
		for (uint16_t i = 0; i <= 180; i += 5) {
			servoWrite(i);
			print("%i\n\r", i);
			delay(50);
		}
	}
	currentCommand = NULL;
}

void testThermo() {
	mrm_therm_b_can.test(userBreak);
	currentCommand = NULL;
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
	currentCommand = &commandDoNothing;
}
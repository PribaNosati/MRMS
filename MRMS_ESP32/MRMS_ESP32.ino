#include <Arduino.h>
#include <BluetoothSerial.h>
#include <mrm-8x8a.h>
#include <mrm-board.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-col-can.h>
#include <mrm-common.h>
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
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
#include <mrm-us.h>
#include <Wire.h>


// Defines

//#define COMMANDS_LIMIT 50 // Increase if more commands are needed
#define LED_ERROR 15 // Pin number, hardware defined
#define MOTOR_GROUP 2 // 0 - Soccer BLDC, 1 - Soccer BDC 2 x mrm-mot2x50, 2 - differential mrm-mot4x3.6, 3 - Soccer BDC mrm-mot4x10, 4 - differential mrm-bldc4x2.5
#define print robot.print


// Commands

struct Command commandCanSniff;
struct Command commandFirmware;
struct Command commandFPS;
struct Command commandGoAhead;
struct Command commandIdChange;
struct Command commandLidarCalibrate;
struct Command commandMenuMain;
struct Command commandMenuPrint;
struct Command commandReflectanceArrayCalibrate;
struct Command commandReportCANBusDevices;
struct Command commandSoccerIdle;
struct Command commandSoccerCatch;
struct Command commandSoccerPlayStart;
struct Command commandLineFollow; // State machine example - following a line
struct Command commandStop; // State machine example - stopped
struct Command commandTest8x8;
struct Command commandTestAll;
struct Command commandTestAny;
struct Command commandTestBluetooth;
struct Command commandTestColor;
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


// Variables

float headingToMaintain; // Soccer


// Objects

BluetoothSerial SerialBT;
MotorGroupDifferential* motorGroupDifferential = NULL;
MotorGroupStar *motorGroupStar = NULL;
Robot robot(&SerialBT);

Mrm_8x8a mrm_8x8a(&robot);
Mrm_bldc4x2_5 mrm_bldc4x2_5(&robot);
Mrm_bldc2x50 mrm_bldc2x50(&robot);
Mrm_col_can mrm_col_can(&robot);
Mrm_imu mrm_imu(&robot);
Mrm_ir_finder2 mrm_ir_finder2(&robot);
Mrm_ir_finder_can mrm_ir_finder_can(&robot);
Mrm_lid_can_b mrm_lid_can_b(&robot, 10);
Mrm_lid_can_b2 mrm_lid_can_b2(&robot);
Mrm_mot2x50 mrm_mot2x50(&robot);//
Mrm_mot4x10 mrm_mot4x10(&robot);
Mrm_mot4x3_6can mrm_mot4x3_6can(&robot);
Mrm_node mrm_node(&robot);
Mrm_ref_can mrm_ref_can(&robot);
Mrm_servo mrm_servo(&robot);
Mrm_switch mrm_switch(&robot);
Mrm_therm_b_can mrm_therm_b_can(&robot);
Mrm_us mrm_us(&robot);
Mrm_pid pidXY(0.5, 200, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane
Mrm_pid pidRotation(0.5, 100, 0); // PID controller, regulates rotation around z axis


// Code

/** Runs once
*/
void setup() {
	Serial.begin(115200);
	SerialBT.begin("ESP32 A"); //Start Bluetooth. ESP32 - Bluetooth device name, choose one.
	Wire.begin(); // Start I2C
	delay(50);
	print("ESP32-Arduino-CAN\r\n");
	initialize();
}

/** Runs constantly
*/
void loop() {
	robot.commandUpdate(mrm_8x8a.alive(), mrm_8x8a.actionCheck(), mrm_switch.actionCheck()); // Check if a key pressed and update current command buffer.
	if (robot.commandCurrent == NULL) // If last command finished, display menu.
		robot.menu();
	else
		robot.commandProcess(); // Process current command. The command will be executed while currentCommand is not NULL. Here state maching processing occurs, too.
	robot.noLoopWithoutThis(); // Receive all CAN Bus messages. This call should be included in any loop, like here.
}

void bluetoothTest() {
	uint32_t startMs = millis();
	while (!userBreak()) {
		print("Time: %i ms.\r\n", millis() - startMs);
		delay(100);
	}
	robot.commandCurrent = NULL;
}

void canBusSniff() {
	robot.canBusSniff();
	robot.commandCurrent = NULL;
}


void canIdChange() {
	robot.canIdChange();
	robot.commandCurrent = NULL;
}

void colorTest() {
	if (commandTestColor.firstProcess)
		mrm_col_can.continuousReadingStart();
	mrm_col_can.test();
}

void commandsAdd() {
	//In all or no menus
	robot.menuAdd(&commandMenuMain, "x", "Escape", &menuMainAndIdle, 2 | 4 | 8 | 16);//2 | 4 | 8 | 16 -> in all menus
	robot.menuAdd(&commandMenuPrint, 0, "Menu print", &menu, 0);//0 -> in no menu

	//Main menu (1)
	robot.menuAdd(&commandTestMotors, "mot", "Test motors", &motorTest, 1);
	robot.menuAdd(&commandTestLidars2m, "li2", "Test lid. 2m", &lidar2mTest, 1);
	robot.menuAdd(&commandTestLidars4m, "li4", "Test lid. 4m", &lidar4mTest, 1);
	robot.menuAdd(&commandTest8x8, "led", "Test 8x8", &led8x8Test, 1);
	robot.menuAdd(&commandTestI2C, "i2c", "Test I2C", &i2cTest, 1);
	robot.menuAdd(&commandTestIMU, "imu", "Test IMU", &imuTest, 1);
	robot.menuAdd(&commandTestColor, "col", "Test color", &colorTest, 1);
	robot.menuAdd(&commandTestIRFinder, "irf", "Test ball analog", &irFinderTest, 1);
	robot.menuAdd(&commandTestIRFinderCan, "irs", "Test ball CAN single", &irFinderTestCan, 1);
	robot.menuAdd(&commandTestIRFinderCanCalculated, "irc", "Test ball CAN calcul", &irFinderTestCanCalculated, 1);
	robot.menuAdd(&commandTestBluetooth, "blt", "Test Bluetooth", &bluetoothTest, 1);
	robot.menuAdd(&commandTestReflectanceArray, "ref", "Test refl. arr.", &reflectanceArrayTest, 1);
	robot.menuAdd(&commandTestNode, "nod", "Test node", &nodeTest, 1);
	robot.menuAdd(&commandTestNodeServos, "nos", "Test node serv.", &nodeServosTest, 1);
	robot.menuAdd(&commandReportCANBusDevices, "can", "Report devices", &devicesScan, 1);
	robot.menuAdd(&commandCanSniff, "sni", "Sniff CAN Bus", &canBusSniff, 1);
	robot.menuAdd(&commandLineFollow, "lin", "FollowLine", &lineFollow, 1);
	robot.menuAdd(&commandStop, "sto", "Stop", &commandStopAll, 1);
	robot.menuAdd(&commandGoAhead, "ahe", "Go ahead", &goAhead, 1);
	robot.menuAdd(&commandTestAny, "any", "Any test", &testAny, 1);
	robot.menuAdd(&commandTestAll, "all", "CAN Bus stress", &testAll, 1);
	robot.menuAdd(&commandTestOmniWheels, "omn", "Test omni wheels", &testOmniWheels, 1);
	robot.menuAdd(&commandReflectanceArrayCalibrate, "cal", "Calibrate refl.", &reflectanceArrayCalibrate, 1);
	robot.menuAdd(&commandTestThermo, "the", "Test thermo", &thermoTest, 1);
	robot.menuAdd(&commandTestServo, "ser", "Test servo", &servoTest, 1);
	robot.menuAdd(&commandLidarCalibrate, "lic", "Cal. lidar", &lidarCalibrate, 1);
	robot.menuAdd(&commandFPS, "fps", "FPS", &fpsPrint, 1);
	robot.menuAdd(&commandFirmware, "fir", "Firmware", &firmwarePrint, 1);
	robot.menuAdd(&commandIdChange, "idc", "Device's id change", &canIdChange, 1);
	robot.menuAdd(&commandSoccerPlayStart, "soc", "Soccer play", &soccerPlayStart, 1);
}

void commandStopAll() {
	robot.stopAll();
	robot.commandCurrent = NULL;
}

void devicesScan() {
	robot.devicesScan(true);

	print("End.\r\n");
	robot.commandCurrent = NULL;
}

void firmwarePrint() {
	robot.firmwarePrint();
	robot.commandCurrent = NULL;
}

void fpsPrint() {
	robot.fpsPrint();
	robot.commandCurrent = NULL;
}

void goAhead() {
	const uint8_t speed = 127;
	if (motorGroupDifferential != NULL)
		motorGroupDifferential->go(30, 30);
	if (motorGroupStar != NULL)
		motorGroupStar->go(speed);
	robot.commandCurrent = NULL;
}

void i2cTest() {
	print("Scanning.\n\r");

	bool any = false;
	for (byte address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address); // Transmission tried
		byte status = Wire.endTransmission(); // Was it successful?
		if (status == 0)
		{
			print("Found at address 0x%02x", address);
			any = true;
		}
		else if (status == 4)
			print("Found at address 0x%02x", address);
	}
	if (!any)
		print("Nothing found.\n\n\r");

	robot.commandCurrent = NULL;
}

void imuTest() {
	mrm_imu.test(userBreak);
	robot.commandCurrent = NULL;
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

	// Robot
	robot.add(&mrm_8x8a);
	robot.add(&mrm_bldc4x2_5);
	robot.add(&mrm_bldc2x50);
	robot.add(&mrm_col_can);
	robot.add(&mrm_ir_finder_can);
	robot.add(&mrm_lid_can_b);
	robot.add(&mrm_lid_can_b2);
	robot.add(&mrm_mot2x50);
	robot.add(&mrm_mot4x10);
	robot.add(&mrm_mot4x3_6can);
	robot.add(&mrm_node);
	robot.add(&mrm_ref_can);
	robot.add(&mrm_therm_b_can);
	robot.add(&mrm_us);

	// LEDs
	pinMode(2, OUTPUT);
	digitalWrite(2, false);
	pinMode(LED_ERROR, OUTPUT); // Error LED
	digitalWrite(15, false);

	// Motor groups
#if MOTOR_GROUP == 0
	motorGroupStar = new MotorGroupStar(&mrm_bldc2x50, 0, &mrm_bldc2x50, 1, &mrm_bldc2x50, 2, &mrm_bldc2x50, 3);
#elif MOTOR_GROUP == 1
	motorGroupStar = new MotorGroupStar(&mrm_mot2x50, 0, &mrm_mot2x50, 1, &mrm_mot2x50, 2, &mrm_mot2x50, 3);
#elif MOTOR_GROUP == 2
	motorGroupDifferential = new MotorGroupDifferential(&mrm_mot4x3_6can, 0, &mrm_mot4x3_6can, 2, &mrm_mot4x3_6can, 1, &mrm_mot4x3_6can, 3);
#elif MOTOR_GROUP == 3
	motorGroupStar = new MotorGroupStar(&mrm_mot4x10, 2, &mrm_mot4x10, 3, &mrm_mot4x10, 0, &mrm_mot4x10, 1);
#else
	motorGroupDifferential = new MotorGroupDifferential(&mrm_bldc2x50, 0, &mrm_bldc2x50, 2, &mrm_bldc2x50, 1, &mrm_bldc2x50, 3);
#endif

	// 8x8 LED
	mrm_8x8a.add("LED8x8_1");

	// Motors mrm-bldc2x50
	mrm_bldc2x50.add(false, "BL2x50-0");
	mrm_bldc2x50.add(false, "BL2x50-1");
	mrm_bldc2x50.add(false, "BL2x50-2");
	mrm_bldc2x50.add(false, "BL2x50-3");

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5.add(false, "BL4x2.5-0");
	mrm_bldc4x2_5.add(false, "BL4x2.5-1");
	mrm_bldc4x2_5.add(false, "BL4x2.5-2");
	mrm_bldc4x2_5.add(false, "BL4x2.5-3");

	// Colors sensors mrm-col-can
	mrm_col_can.add("Col-0");
	mrm_col_can.add("Col-1");

	// IMU
	mrm_imu.add();

	// mrm-ir-finder2
	mrm_ir_finder2.add(34, 33);

	// mrm-ir-finder-can
	mrm_ir_finder_can.add("IRFind-0");

	// Motors mrm-mot2x50
	mrm_mot2x50.add(false, "Mot2x50-0");
	mrm_mot2x50.add(false, "Mot2x50-1");
	mrm_mot2x50.add(false, "Mot2x50-2");
	mrm_mot2x50.add(false, "Mot2x50-3");

	// Motors mrm-mot4x10
	mrm_mot4x10.add(true, "Mot4x10-0");
	mrm_mot4x10.add(true, "Mot4x10-1");
	mrm_mot4x10.add(true, "Mot4x10-2");
	mrm_mot4x10.add(true, "Mot4x10-3");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can.add(false, "Mot3.6-0");
	mrm_mot4x3_6can.add(false, "Mot3.6-1");
	mrm_mot4x3_6can.add(true, "Mot3.6-2");
	mrm_mot4x3_6can.add(true, "Mot3.6-3");

	mrm_mot4x3_6can.add(false, "Mot3.6-4");
	mrm_mot4x3_6can.add(false, "Mot3.6-5");
	mrm_mot4x3_6can.add(true, "Mot3.6-6");
	mrm_mot4x3_6can.add(true, "Mot3.6-7");

	// Lidars mrm-lid-can-b, VL53L0X, 2 m
	mrm_lid_can_b.add("Lidar2m-0");
	mrm_lid_can_b.add("Lidar2m-1");
	mrm_lid_can_b.add("Lidar2m-2");
	mrm_lid_can_b.add("Lidar2m-3");
	mrm_lid_can_b.add("Lidar2m-4");
	mrm_lid_can_b.add("Lidar2m-5");
	mrm_lid_can_b.add("Lidar2m-6");
	mrm_lid_can_b.add("Lidar2m-7");
	mrm_lid_can_b.add("Lidar2m-8");
	mrm_lid_can_b.add("Lidar2m-9");

	// Lidars mrm-lid-can-b2, VL53L1X, 4 m
	mrm_lid_can_b2.add("Lidar4m-0");
	mrm_lid_can_b2.add("Lidar4m-1");
	mrm_lid_can_b2.add("Lidar4m-2");
	mrm_lid_can_b2.add("Lidar4m-3");
	mrm_lid_can_b2.add("Lidar4m-4");
	mrm_lid_can_b2.add("Lidar4m-5");
	mrm_lid_can_b2.add("Lidar4m-6");
	mrm_lid_can_b2.add("Lidar4m-7");

	// CAN Bus node
	mrm_node.add("Node-0");
	mrm_node.add("Node-1");

	// Reflective array
	mrm_ref_can.add("RefArr-0");
	mrm_ref_can.add("RefArr-1");
	mrm_ref_can.add("RefArr-2");
	mrm_ref_can.add("RefArr-3");

	// Servo motors
	mrm_servo.add(16, "Servo", 10);

	// Switch
	mrm_switch.add(18, 19, "Switch");
	mrm_switch.actionSet(&commandTestAny, 0);
	mrm_switch.actionSet(&commandStop, 1);

	// Thermal array
	mrm_therm_b_can.add("Thermo-0");
	mrm_therm_b_can.add("Thermo-1");
	mrm_therm_b_can.add("Thermo-2");
	mrm_therm_b_can.add("Thermo-3");

	// Ultrasonic
	mrm_us.add("US-0");
	mrm_us.add("US-1");
	mrm_us.add("US-2");
	mrm_us.add("US-3");

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

	robot.commandCurrent = NULL;
}

void lineFollow() {
	static float lastLineCenter = 0;
	//static uint32_t ms = 0;
	if (commandLineFollow.firstProcess) // Only in the first pass.
		robot.broadcastingStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.

	float lineCenter = (mrm_ref_can.center() - 4500) / 70.0; // Result: -50 to 50.
	//if (millis() - ms > 100) {
	//	print("C:%i R:%i %i %i %i %i %i %i %i %i %ims\n\r", (int)lineCenter, (int)mrm_ref_can.center(), (int)mrm_ref_can.dark(7),
	//		mrm_ref_can.dark(6), mrm_ref_can.dark(5), mrm_ref_can.dark(4), mrm_ref_can.dark(3), mrm_ref_can.dark(2), mrm_ref_can.dark(1),
	//		mrm_ref_can.dark(0), millis() - mrm_ref_can.lastMessageMs());
	//	ms = millis();
	//}
	if (lineCenter < -40 || lineCenter > 40) {// No line under inner sensors, including lost line.
		//motorGroupDifferential->stop();
		//delay(100);
		// Choice depending on the line center the last time it was detected, stored in variable lastLineCenter.
		if (lastLineCenter < -15) // Lost line right or line far right.
			motorGroupDifferential->go(127, -127); // Rotate in place.
		else if (lastLineCenter > 15) // Lost line left or line far left.
			motorGroupDifferential->go(-127, 127); // Rotate in place.
		else // Line was around the robot's center when lost. Therefore, it was interrupted
			motorGroupDifferential->go(127, 127); // Go straight ahead.
	}
	else { // Follow line
		// Maximum speed of the faster motor, decrease the other one.
		motorGroupDifferential->go(lineCenter < 0 ? 127 : 127 - lineCenter * 3, lineCenter < 0 ? 127 + lineCenter * 3 : 127);
		lastLineCenter = lineCenter; // Remember the line.
	}
}

void menu() {
	robot.menu();
}

void menuMainAndIdle() {
	commandStopAll();

	robot.menuLevel = 1;
	robot.commandCurrent = NULL;
}

void motorTest() {
	robot.motorTest();
	robot.commandCurrent = NULL;
}

void nodeServosTest() {
	mrm_node.servoTest(userBreak);
}

void nodeTest() {
	if (commandTestNode.firstProcess)
		mrm_node.continuousReadingStart();
	mrm_node.test();
}

void reflectanceArrayCalibrate() {
	mrm_ref_can.calibrate();
	robot.commandCurrent = NULL;
}

void reflectanceArrayTest() {
	if (commandTestReflectanceArray.firstProcess) 
		mrm_ref_can.continuousReadingStart();
	mrm_ref_can.test();
}

void servoTest() {
	mrm_servo.test(userBreak);
	robot.commandCurrent = NULL;
}

void soccerCatch() {
	if (mrm_ir_finder2.anyIRSource()) {

	}
	else
		robot.commandSet(&commandSoccerIdle);
}

void soccerIdle() {
	static float headingToMaintain;
	if (commandSoccerIdle.firstProcess)
		headingToMaintain = mrm_imu.heading();
	if (mrm_ir_finder2.anyIRSource() && false)
		robot.commandSet(&commandSoccerCatch);
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
	robot.broadcastingStart();
	robot.commandSet(&commandSoccerIdle);
}

void testAll() {
	if (robot.stressTest(commandTestAll.firstProcess))
		robot.commandCurrent = NULL;
}

void testAny() {
	
}

void testOmniWheels() {
	static uint8_t nextMove;
	static uint32_t lastMs;
	if (commandTestOmniWheels.firstProcess) {
		if (motorGroupDifferential == NULL) {
			print("Differential motor group needed.");
			robot.commandCurrent = NULL;
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

bool userBreak() { // todo - should be deleted and robot.userBreak() used instead.
	if (/*switchOn() ||*/ Serial.available() || SerialBT.available()) {
		return true;
	}
	else
		return false;
}
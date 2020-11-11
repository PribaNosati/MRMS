#include <mrm-board.h>
#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-can-bus.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-pid.h>
#include <mrm-ir-finder2.h>
#include <mrm-ir-finder-can.h>
#include <mrm-ir-finder3.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-ref-can.h>
#include <mrm-robot.h>
#include <mrm-servo.h>
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
#include <mrm-us.h>


extern BluetoothSerial* serialBT;

/**
*/
Robot::Robot(char name[15]) {
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
	Serial.begin(115200);
	//serial = new BluetoothSerial();
	if (strlen(name) > 15)
		strcpy(errorMessage, "Name overflow");
	strcpy(_name, name);
	if (serialBT == NULL) {
		serialBT = new BluetoothSerial(); // Additional serial port
		serialBT->begin(_name); //Start Bluetooth. ESP32 - Bluetooth device name, choose one.
	}

	delay(50);
	print("%s started.\r\n", _name);

	Wire.begin(); // Start I2C

	mrm_can_bus = new Mrm_can_bus();

	_actionCurrent = NULL;
	_actionPrevious = _actionCurrent;

	_actionAny = new ActionAny(this);
	_actionCANBusStress = new ActionCANBusStress(this);
	_actionDoNothing = new ActionDoNothing(this);
	_actionMenuMain = new ActionMenuMain(this);
	_actionStop = new ActionStop(this);

	actionAdd(new Action8x8Test(this));
	actionAdd(_actionAny);
	actionAdd(new ActionBluetoothTest(this));
	actionAdd(new ActionCANBusScan(this));
	actionAdd(new ActionCANBusSniff(this));
	actionAdd(new ActionCANBusStress(this));
	actionAdd(new ActionColorIlluminationOff(this));
	actionAdd(new ActionColorIlluminationOn(this));
	actionAdd(new ActionColorPatternErase(this));
	actionAdd(new ActionColorPatternPrint(this));
	actionAdd(new ActionColorPatternRecognize(this));
	actionAdd(new ActionColorPatternRecord(this));
	actionAdd(new ActionColorTest6Colors(this));
	actionAdd(new ActionColorTestHSV(this));
	actionAdd(new ActionDeviceIdChange(this));
	actionAdd(new ActionFirmware(this));
	actionAdd(new ActionFPS(this));
	actionAdd(new ActionGoAhead(this));
	actionAdd(new ActionI2CTest(this));
	actionAdd(new ActionIMUTest(this));
	actionAdd(new ActionInfo(this));
	actionAdd(new ActionIRFinderTest(this));
	actionAdd(new ActionIRFinderCanTest(this));
	actionAdd(new ActionIRFinderCanTestCalculated(this));
	actionAdd(new ActionLidar2mTest(this));
	actionAdd(new ActionLidar4mTest(this));
	actionAdd(new ActionLidarCalibrate(this));
	actionAdd(new ActionMenuColor(this));
	actionAdd(new ActionMenuMain(this));
	actionAdd(new ActionMenuReflectance(this));
	actionAdd(new ActionMenuSystem(this));
	actionAdd(new ActionMotorTest(this));
	actionAdd(new ActionNodeTest(this));
	actionAdd(new ActionNodeServoTest(this));
	//actionAdd(new ActionOscillatorTest(this));
	actionAdd(new ActionReflectanceArrayCalibrate(this));
	actionAdd(new ActionReflectanceArrayCalibrationPrint(this));
	actionAdd(new ActionReflectanceArrayAnalogTest(this));
	actionAdd(new ActionReflectanceArrayDigitalTest(this));
	actionAdd(new ActionServoInteractive(this));
	actionAdd(new ActionServoTest(this));
	actionAdd(_actionStop);
	actionAdd(new ActionThermoTest(this));

	mrm_8x8a = new Mrm_8x8a(this);
	mrm_bldc2x50 = new Mrm_bldc2x50(this);
	mrm_bldc4x2_5 = new Mrm_bldc4x2_5(this);
	mrm_col_can = new Mrm_col_can(this);
	mrm_imu = new Mrm_imu(this);
	mrm_ir_finder2 = new Mrm_ir_finder2(this);
	mrm_ir_finder_can = new Mrm_ir_finder_can(this);
	mrm_ir_finder3 = new Mrm_ir_finder3(this);
	mrm_lid_can_b = new Mrm_lid_can_b(this);
	mrm_lid_can_b2 = new Mrm_lid_can_b2(this);
	mrm_mot2x50 = new Mrm_mot2x50(this);
	mrm_mot4x3_6can = new Mrm_mot4x3_6can(this);
	mrm_mot4x10 = new Mrm_mot4x10(this);
	mrm_node = new Mrm_node(this);
	mrm_ref_can = new Mrm_ref_can(this);
	mrm_servo = new Mrm_servo(this);
	mrm_switch = new Mrm_switch(this);
	mrm_therm_b_can = new Mrm_therm_b_can(this);
	mrm_us = new Mrm_us(this);

	// 8x8 LED
	mrm_8x8a->add("LED8x8-0");

	// Motors mrm-bldc2x50
	mrm_bldc2x50->add(false, "BL2x50-0");
	mrm_bldc2x50->add(false, "BL2x50-1");
	mrm_bldc2x50->add(false, "BL2x50-2");
	mrm_bldc2x50->add(false, "BL2x50-3");

	// LEDs
	pinMode(2, OUTPUT);
	digitalWrite(2, false);
	pinMode(LED_ERROR, OUTPUT); // Error LED
	digitalWrite(15, false);

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5->add(false, "BL4x2.5-0");
	mrm_bldc4x2_5->add(false, "BL4x2.5-1");
	mrm_bldc4x2_5->add(false, "BL4x2.5-2");
	mrm_bldc4x2_5->add(false, "BL4x2.5-3");

	// Colors sensors mrm-col-can
	mrm_col_can->add("Col-0");
	mrm_col_can->add("Col-1");
	mrm_col_can->add("Col-2");
	mrm_col_can->add("Col-3");

	// IMU
	mrm_imu->add();

	// mrm-ir-finder2
	mrm_ir_finder2->add(34, 33);

	// mrm-ir-finder-can
	mrm_ir_finder_can->add("IRFind-0");

	// mrm-ir-finder3
	mrm_ir_finder3->add("IR3Fin-0");

	// Motors mrm-mot2x50
	mrm_mot2x50->add(false, "Mot2x50-0");
	mrm_mot2x50->add(false, "Mot2x50-1");
	mrm_mot2x50->add(false, "Mot2x50-2");
	mrm_mot2x50->add(false, "Mot2x50-3");
	mrm_mot2x50->add(false, "Mot2x50-4");
	mrm_mot2x50->add(false, "Mot2x50-5");

	// Motors mrm-mot4x10
	mrm_mot4x10->add(false, "Mot4x10-0");
	mrm_mot4x10->add(false, "Mot4x10-1");
	mrm_mot4x10->add(false, "Mot4x10-2");
	mrm_mot4x10->add(false, "Mot4x10-3");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can->add(false, "Mot3.6-0");
	mrm_mot4x3_6can->add(false, "Mot3.6-1");
	mrm_mot4x3_6can->add(false, "Mot3.6-2");
	mrm_mot4x3_6can->add(false, "Mot3.6-3");

	mrm_mot4x3_6can->add(false, "Mot3.6-4");
	mrm_mot4x3_6can->add(false, "Mot3.6-5");
	mrm_mot4x3_6can->add(false, "Mot3.6-6");
	mrm_mot4x3_6can->add(false, "Mot3.6-7");

	// Lidars mrm-lid-can-b, VL53L0X, 2 m
	mrm_lid_can_b->add("Lidar2m-0");
	mrm_lid_can_b->add("Lidar2m-1");
	mrm_lid_can_b->add("Lidar2m-2");
	mrm_lid_can_b->add("Lidar2m-3");
	mrm_lid_can_b->add("Lidar2m-4");
	mrm_lid_can_b->add("Lidar2m-5");
	mrm_lid_can_b->add("Lidar2m-6");
	mrm_lid_can_b->add("Lidar2m-7");
	mrm_lid_can_b->add("Lidar2m-8");
	mrm_lid_can_b->add("Lidar2m-9");
	mrm_lid_can_b->add("Lidar2m10");
	mrm_lid_can_b->add("Lidar2m11");	
	mrm_lid_can_b->add("Lidar2m12");
	mrm_lid_can_b->add("Lidar2m13");

	// Lidars mrm-lid-can-b2, VL53L1X, 4 m
	mrm_lid_can_b2->add("Lidar4m-0");
	mrm_lid_can_b2->add("Lidar4m-1");
	mrm_lid_can_b2->add("Lidar4m-2");
	mrm_lid_can_b2->add("Lidar4m-3");
	mrm_lid_can_b2->add("Lidar4m-4");
	mrm_lid_can_b2->add("Lidar4m-5");
	mrm_lid_can_b2->add("Lidar4m-6");
	mrm_lid_can_b2->add("Lidar4m-7");

	// CAN Bus node
	mrm_node->add("Node-0");
	mrm_node->add("Node-1");

	// Reflective array
	mrm_ref_can->add("RefArr-0");
	mrm_ref_can->add("RefArr-1");
	mrm_ref_can->add("RefArr-2");
	mrm_ref_can->add("RefArr-3");

	// Servo motors. Note that some pins are not appropriate for PWM (servo)
	mrm_servo->add(18, "Servo1", 0, 300, 0.5, 2.5); // Data for mrm-rds5060-300
	mrm_servo->add(19, "Servo2", 0, 300, 0.5, 2.5);
	mrm_servo->add(16, "Servo3", 0, 300, 0.5, 2.5); 
	mrm_servo->add(17, "Servo4", 0, 300, 0.5, 2.5);


	// Switch
	mrm_switch->add(18, 19, "Switch");

	// Thermal array
	mrm_therm_b_can->add("Thermo-0");
	mrm_therm_b_can->add("Thermo-1");
	mrm_therm_b_can->add("Thermo-2");
	mrm_therm_b_can->add("Thermo-3");

	// Ultrasonic
	mrm_us->add("US-0");
	mrm_us->add("US-1");
	mrm_us->add("US-2");
	mrm_us->add("US-3");

	// Add boards
	add(mrm_8x8a);
	add(mrm_bldc2x50);
	add(mrm_bldc4x2_5);
	add(mrm_col_can);
	add(mrm_ir_finder3);
	add(mrm_lid_can_b);
	add(mrm_lid_can_b2);
	add(mrm_mot2x50);
	add(mrm_mot4x10);
	add(mrm_mot4x3_6can);
	add(mrm_node);
	add(mrm_ref_can);
	add(mrm_therm_b_can);
	add(mrm_us);
}

/** Add a new action to the collection of robot's possible actions.
@param action - the new action.
*/
void Robot::actionAdd(ActionBase* action) {
	if (_actionNextFree >= ACTIONS_LIMIT) {
		strcpy(errorMessage, "ACTIONS_LIMIT exceeded.");
		return;
	}
	_action[_actionNextFree++] = action;
}

/** Actually perform the action
*/
void Robot::actionProcess() {
	if (_actionCurrent != NULL) {
		if (_actionCurrent->preprocessing()) {
			_actionCurrent->performBefore();
		}
		_actionCurrent->perform();
		//if (_actionCurrent != NULL)
		//	_actionCurrent->_preprocessing = false;
	}
}

/** User sets a new action, using keyboard or Bluetooth
*/
void Robot::actionSet() {
	static uint32_t lastUserActionMs = 0;
	static uint8_t uartRxCommandIndex = 0;
	static char uartRxCommandCumulative[10];
	const uint16_t TIMEOUT_MS = 2000;

	// If a button pressed, first execute its action
	ActionBase* action8x8 = mrm_8x8a->actionCheck();
	ActionBase* actionSw = mrm_switch->actionCheck();
	if (action8x8 != NULL)
		_actionCurrent = action8x8;
	else if (actionSw != NULL)
		_actionCurrent = actionSw;
	else { // Check keyboard
		if (Serial.available() || serialBT != NULL && serialBT->available()) {
			lastUserActionMs = millis();
			uint8_t ch;
			if (Serial.available())
				ch = Serial.read();
			else
				if (serialBT != NULL)
					ch = serialBT->read();

			if (ch != 13) //if received data different from ascii 13 (enter)
				uartRxCommandCumulative[uartRxCommandIndex++] = ch;	//add data to Rx_Buffer

			if (ch == 13 || uartRxCommandIndex >= 3 || ch == 'x') //if received data = 13
			{
				uartRxCommandCumulative[uartRxCommandIndex] = 0;
				uartRxCommandIndex = 0;

				print("Command: %s", uartRxCommandCumulative);

				uint8_t found = 0;
				for (uint8_t i = 0; i < _actionNextFree; i++) {
					if (strcmp(_action[i]->_shortcut, uartRxCommandCumulative) == 0) {
						print(" ok.\r\n");
						actionSet(_action[i]);
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
}

/** New action is set in the program
@param newAction - the new action.
*/
void Robot::actionSet(ActionBase* newAction) {
	_actionPrevious = _actionCurrent;
	_actionCurrent = newAction;
	_actionCurrent->preprocessingStart();
}

/** Add a new board to the collection of possible boards for the robot
@param aBoard - the board.
*/
void Robot::add(Board* aBoard) {
	if (_boardNextFree > BOARDS_LIMIT - 1) {
		strcpy(errorMessage, "Too many boards");
		return;
	}
	board[_boardNextFree++] = aBoard;
}

/** Blink LED
*/
void Robot::blink() {
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

/** Displays all boards
@return - last board and device's index, 0 if none
*/
uint8_t Robot::boardsDisplayAll() {
	// Print all devices alive
	uint8_t last = 0;
	for (uint8_t boardNumber = 0; boardNumber < _boardNextFree; boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount(); deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0)
					print("%i.", ++last);
				else
					print(",");
				print(" %s", board[boardNumber]->name(deviceNumber));
				if (++currentCount == board[boardNumber]->devicesOnASingleBoard()) {
					currentCount = 0;
					print("\n\r");
				}
			}
	}
	if (last == 0)
		print("No boards\n\r");
	return last;
}

/** Finds board and device's index. Similar to next function, but display choices, too.
@param selectedBoardIndex - output
@param selectedDeviceIndex - otuput
@param maxInput - output
@param lastBoardAndIndex - output
@return - true if found
*/
bool Robot::boardDisplayAndSelect(uint8_t *selectedBoardIndex, uint8_t* selectedDeviceIndex, uint8_t* maxInput, uint8_t* lastBoardAndIndex) {
	*lastBoardAndIndex = boardsDisplayAll();
	bool found = false;
	if (*lastBoardAndIndex > 0) {

		// Choose device 
		print("Enter board [1 - %i]: ", *lastBoardAndIndex);
		uint16_t selectedNumber = serialReadNumber(15000, 500, *lastBoardAndIndex <= 9, *lastBoardAndIndex);
		print("%i", selectedNumber);

		if (selectedNumber != 0xFFFF)
			found = boardSelect(selectedNumber, selectedBoardIndex, selectedDeviceIndex, maxInput);

	}
	return found;
}

/** Finds board and device's index for a number received from boardsDisplayAll()
@param selectedNumber - input
@param selectedBoardIndex - output
@param selectedDeviceIndex - otuput
@param maxInput - output
@return - true if found
*/
bool Robot::boardSelect(uint8_t selectedNumber, uint8_t *selectedBoardIndex, uint8_t * selectedDeviceIndex, uint8_t *maxInput) {
	// Find selected board
	uint8_t lastBoardAndIndex = 0;
	*selectedBoardIndex = 0;
	*selectedDeviceIndex = 0xFF;
	*maxInput = 0;
	for (uint8_t boardNumber = 0; boardNumber < _boardNextFree && *selectedDeviceIndex == 0xFF; boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount() && *selectedDeviceIndex == 0xFF; deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0) {
					if (++lastBoardAndIndex == selectedNumber) {
						Board *selectedBoard = board[boardNumber];
						*selectedDeviceIndex = deviceNumber;
						*selectedBoardIndex = boardNumber;
						*maxInput = board[boardNumber]->deadOrAliveCount() / board[boardNumber]->devicesOnASingleBoard() - 1;
						break;
					}
				}

				if (++currentCount == board[boardNumber]->devicesOnASingleBoard())
					currentCount = 0;
			}
	}
	return *selectedDeviceIndex != 0xFF;
}

/** Test Bluetooth
*/
void Robot::bluetoothTest() {
	static uint32_t startMs = millis();
	if (millis() - startMs > 100) {
		print("Time: %i ms.\r\n", millis() - startMs);
		startMs = millis();
	}
}

/** Display all the incomming and outcomming CAN Bus messages
*/
void Robot::canBusSniffToggle() {
	_sniff = !_sniff;
	if (_sniff)
		print("Sniff on\n\r");
	else
		print("Sniff off\n\r");
	actionEnd();
}

/** Change device's id
*/
void Robot::canIdChange() {
	uint8_t selectedBoardIndex;
	uint8_t selectedDeviceIndex;
	uint8_t maxInput;
	uint8_t lastBoardAndDeviceIndex;
	if (boardDisplayAndSelect(&selectedBoardIndex, &selectedDeviceIndex, &maxInput, &lastBoardAndDeviceIndex)) {
		// Enter new id
		print(". %s\n\rEnter new board id [0..%i]: ", board[selectedBoardIndex]->name(), maxInput);
		uint8_t newDeviceNumber = serialReadNumber(15000, 500, maxInput <= 9, maxInput);

		if (newDeviceNumber != 0xFF) {
			// Change
			print("%i\n\rChange requested.\n\r", newDeviceNumber);
				board[selectedBoardIndex]->idChange(newDeviceNumber, selectedDeviceIndex);
			delayMs(500); // Delay for firmware handling of devices with the same ids.
		}
	}

	actionEnd();
}

/** mrm-color-can illumination off
*/
void Robot::colorIlluminationOff() {
	mrm_col_can->illumination(0xFF, 0);
	actionEnd();
}

/** mrm-color-can illumination on
*/
void Robot::colorIlluminationOn() {
	mrm_col_can->illumination(0xFF, 1);
	actionEnd();
}

/** Erase HSV patterns
*/
void Robot::colorPatternErase() {
	mrm_col_can->patternErase();
	actionEnd();
}

/** Print HSV patterns
*/
void Robot::colorPatternPrint() {
	mrm_col_can->patternPrint();
	actionEnd();
}

/** Record HSV color patterns
*/
void Robot::colorPatternRecord() {
	mrm_col_can->patternsRecord();
	actionEnd();
}

/** Recognize HSV color pattern
*/
void Robot::colorPatternRecognize() {
	actionEnd();
}

/** The right way to use Arduino function delay
@param pauseMs - pause in ms. One run even if pauseMs == 0, so that delayMs(0) receives all messages.
*/
void Robot::delayMs(uint16_t pauseMs) {
	uint32_t startMs = millis();
	do  {
		noLoopWithoutThis();
	} while (millis() - startMs < pauseMs);
}

/** The right way to use Arduino function delayMicros
@param pauseMicros - pause in micros. One run even if pauseMicros == 0, so that delayMicross(0) receives all messages.
*/
void Robot::delayMicros(uint16_t pauseMicros) {
	uint32_t startMicros = micros();
	do {
		noLoopWithoutThis();
	} while (micros() - startMicros < pauseMicros);
}

/** Contacts all the CAN Bus devices and checks which one is alive.
@verbose - if true, print. 
@return count
*/
uint8_t Robot::devicesScan(bool verbose) {
	devicesStop();
	uint8_t count = 0;
	delayMs(100); // Read all the messages sent after stop.
	for (uint8_t i = 0; i < _boardNextFree; i++)
		count += board[i]->devicesScan(verbose);
	if (verbose)
		print("%i devices.\n\r", count);
	actionEnd();
	return count;
}

/** Starts devices' CAN Bus messages broadcasting.
*/
void Robot::devicesStart(uint8_t measuringMode) {
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++)
		board[deviceNumber]->start(0xFF, measuringMode);
}

/** Stops broadcasting of CAN Bus messages
*/
void Robot::devicesStop() {
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++) {
		board[deviceNumber]->stop();
		delayMs(1); // TODO
	}
}

/** Displays errors and stops motors, if any.
*/
void Robot::errors() {
	if (strcmp(errorMessage, "") != 0) {
		print("Error! %s\n\r", errorMessage);
		strcpy(errorMessage, "");
		stopAll(); // Stop all motors
		actionEnd();
	}
}

/** Displays each CAN Bus device's firmware
*/
void Robot::firmwarePrint() {
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->firmwareRequest();
		uint32_t startMs = millis();
		delayMs(1);
	}
	actionEnd();
}

/** Returns FPS (frames per second).
@return - FPS
*/
float Robot::fpsGet() {
	float fpsNow;
	//print("Next: %i %i %i\n\r", fpsNextIndex, fpsMs[0], fpsMs[1]);
	if (fpsMs[1] == 0 || fpsMs[0] == 0)
		fpsNow = 0;
	else if (fpsMs[0] == fpsMs[1])
		fpsNow = 1000000;
	else
		fpsNow = 1000000.0 / (float)(fpsMs[1] > fpsMs[0] ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
	return fpsNow;
}

/** Avoids FPS measuring in the next 2 cycles.
*/
void Robot::fpsPause() {
	fpsMs[0] = 0;
	fpsMs[1] = 0;
}

/** Prints FPS all CAN Bus devices and mrm-eps32 boards. Also prints CAN Bus frequency.
*/
void Robot::fpsPrint() {
	print("CAN peaks: %i received/s, %i sent/s\n\r", mrm_can_bus->messagesPeakReceived(), mrm_can_bus->messagesPeakSent());
	print("Arduino: %i FPS, low peak: %i FPS\n\r", (int)fpsGet(), fpsTopGap == 1000000 ? 0 : (int)(1000000 / (float)fpsTopGap));
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->fpsRequest();
		uint32_t startMs = millis();
		while (millis() - startMs < 30)
			noLoopWithoutThis();
		board[i]->fpsDisplay();
	}
	fpsReset();
	actionEnd();
}

/** Resets FPS data
*/
void Robot::fpsReset() {
	fpsTopGap = 0;
	fpsMs[0] = 0;
	fpsMs[1] = 0;
	mrm_can_bus->messagesReset();
}

/** Updates data for FPS calculation
*/
void Robot::fpsUpdate() {
	fpsMs[fpsNextIndex] = micros();
	fpsNextIndex = (fpsNextIndex == 0 ? 1 : 0);
	if (fpsMs[0] != 0 && fpsMs[1] != 0) {
		uint32_t gap = (fpsNextIndex == 0 ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
		if (gap > fpsTopGap)
			fpsTopGap = gap;
	}
}

/** Lists I2C devices
*/
void Robot::i2cTest() {
	print("Scanning.\n\r");

	bool any = false;
	for (byte address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address); // Transmission tried
		byte status = Wire.endTransmission(); // Was it successful?
		if (status == 0)
		{
			print("Found at address 0x%02x\n\r", address);
			any = true;
		}
		else if (status == 4)
			print("Found at address 0x%02x\n\r", address);
	}
	if (!any)
		print("Nothing found.\n\n\r");

	actionEnd();
}

/** Request information
*/
void Robot::info() {
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->info();
		delay(1);
	}
	actionEnd();
}

/** Tests mrm-ir-finder-can, raw data.
*/
void Robot::irFinder3Test() {
	if (actionPreprocessing(true))
		mrm_ir_finder3->start();
	mrm_ir_finder3->test();
}

/** Tests mrm-ir-finder-can, calculated data.
*/
void Robot::irFinder3TestCalculated() {
	if (actionPreprocessing(true))
		mrm_ir_finder3->continuousReadingCalculatedDataStart();
	mrm_ir_finder3->testCalculated();
}

/** Tests mrm-lid-can-b
*/
void Robot::lidar2mTest() {
	static uint16_t selected;
	if (actionPreprocessing(true)) {
		// Select lidar
		uint8_t count = mrm_lid_can_b->deadOrAliveCount();
		print("%s - enter lidar number [0-%i] or wait for all\n\r", mrm_lid_can_b->name(), count - 1);
		selected = serialReadNumber(3000, 1000, count - 1 < 9, count - 1, false);
		if (selected == 0xFFFF) {
			print("Test all\n\r");
			selected = 0xFF;
		}
		else {
			if (mrm_lid_can_b->alive(selected))
				print("\n\rTest lidar %s\n\r", mrm_lid_can_b->name(selected));
			else {
				print("\n\rLidar %s dead, test all\n\r", mrm_lid_can_b->name(selected));
				selected = 0xFF;
			}
		}
		//mrm_lid_can_b->start(selected);
	}
	mrm_lid_can_b->test(selected);
}

/** Tests mrm-lid-can-b2
*/
void Robot::lidar4mTest() {
	if (actionPreprocessing(true))
		mrm_lid_can_b2->start();
	mrm_lid_can_b2->test();
}

/** Calibrates lidars
*/
void Robot::lidarCalibrate() {
	print("Lidar calibration\n\r");

	// Select lidar 2 or 4 m
	int8_t selected2Or4 = -1;
	uint32_t lastMs;
	while (selected2Or4 != 2 && selected2Or4 != 4) {
		print("Enter max distance [2 or 4]m or wait to abort ");
		lastMs = millis();
		selected2Or4 = -1;
		while (millis() - lastMs < 10000 && selected2Or4 == -1)
			if (Serial.available()) {
				uint8_t ch = Serial.read() - 48;
				print("%i\n\r", ch);
				selected2Or4 = ch;
			}
		if (selected2Or4 == -1) {
			print("- abort\n\r");
			break;
		}
	}

	// Select lidar number
	if (selected2Or4 != -1) {
		// Select lidar
		uint8_t count = selected2Or4 == 2 ? mrm_lid_can_b->deadOrAliveCount() : mrm_lid_can_b2->deadOrAliveCount();
		print("Enter lidar number [0-%i] or wait to abort", count - 1);
		uint16_t selected = serialReadNumber(3000, 1000, count - 1 < 9, count - 1, false);
		if (selected == 0xFFFF)
			print("\n\rAbort\n\r");
		else {
			if (selected2Or4 == 2 ? mrm_lid_can_b->alive(selected) : mrm_lid_can_b2->alive(selected)) {
				print("\n\rCalibrate lidar %s\n\r", mrm_lid_can_b->name(selected));
				selected2Or4 == 2 ? mrm_lid_can_b->calibration(selected) : mrm_lid_can_b2->calibration(selected);
			}
			else
				print("\n\rLidar %s dead\n\r", selected2Or4 == 2 ? mrm_lid_can_b->name(selected) : mrm_lid_can_b2->name(selected));
		}
	}

	actionEnd();
}

/** Displays menu
*/
void Robot::menu() {
	// Print menu
	if (_devicesScanBeforeMenu)
		devicesScan(false);
	print("\r\n");

	bool any = false;
	uint8_t column = 1;
	uint8_t maxColumns = 2;
	for (uint8_t i = 0; i < _actionNextFree; i++) {
		if ((_action[i]->_menuLevel | menuLevel) == _action[i]->_menuLevel) {
			bool anyAlive = false;
			if (_action[i]->boardsId() == ID_ANY)
				anyAlive = true;
			else
				for (uint8_t j = 0; j < _boardNextFree && !anyAlive; j++)
					if (board[j]->alive(0xFF) && board[j]->id() == _action[i]->boardsId())
						anyAlive = true;
			if (anyAlive) {
				print("%-3s - %-19s%s", _action[i]->_shortcut, _action[i]->_text, column == maxColumns ? "\n\r" : "");
				delayMs(2);
				any = true;
				if (column++ == maxColumns)
					column = 1;
			}
		}
	}

	if (!any)
		print("Menu level %i empty.\r\n", menuLevel);
	else
		if (column != 1)

	// Display errors
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++)
		if (board[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", board[deviceNumber]->errorCodeLast(), board[deviceNumber]->name(board[deviceNumber]->errorWasInDeviceNumber()));

	fpsPause(); // this function took too much time

	_actionCurrent = _actionDoNothing;
}

/** Color menu
*/
void Robot::menuColor() {
	menuLevel = 4;
	actionEnd();
}

/** Displays menu and stops motors
*/
void Robot::menuMainAndIdle() {
	stopAll();
	menuLevel = 1;
}

/** Reflectance menu
*/
void Robot::menuReflectance() {
	menuLevel = 2;
	actionEnd();
}

/** System menu
*/
void Robot::menuSystem() {
	menuLevel = 16;
	actionEnd();
}

/** Print CAN Bus message
@param msg - message
@param oubound - if not, inbound
*/
void Robot::messagePrint(CANBusMessage *msg, bool outbound) {
	bool any = false;
	for (uint8_t boardId = 0; boardId < _boardNextFree; boardId++) {
		if (board[boardId]->messagePrint(msg->messageId, msg->dlc, msg->data, outbound)) {
			any = true;
			break;
		}
	}
	if (!any) {
		print("Id:0x%02X", msg->messageId);
		for (uint8_t i = 0; i < msg->dlc; i++) {
			if (i == 0)
				print(" data:");
			print(" %02X", msg->data[i]);
		}
		print("\n\r");
	}
}

/** Receives CAN Bus messages. 
*/
void Robot::messagesReceive() {
	while (true) {
		CANBusMessage* msg = mrm_can_bus->messageReceive();
		if (msg == NULL) // No more messages
			break;
		uint32_t id = msg->messageId;
		if (_sniff)
			messagePrint(msg, false);
		bool any = false;
		for (uint8_t boardId = 0; boardId < _boardNextFree; boardId++) {
			if (board[boardId]->messageDecode(id, msg->data)) {
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

/** Tests motors
*/
void Robot::motorTest() {
	print("Test motors\n\r");
	for (uint8_t i = 0; i < _boardNextFree; i++) 
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->count() > 0)
			board[i]->test();
	actionEnd();
}

/** Tests mrm-node
*/
void Robot::nodeTest() {
	if (actionPreprocessing(true))
		mrm_node->start();
	mrm_node->test();
}

/** Any for or while loop must include call to this function.
*/
void Robot::noLoopWithoutThis() {
	blink(); // Keep-alive LED. Solder jumper must be shorted in order to work in mrm-esp32.
	messagesReceive();
	fpsUpdate(); // Measure FPS. Less than 30 - a bad thing.
	verbosePrint(); // Print FPS and maybe some additional data
	errors();
}

/** Production test
*/
void Robot::oscillatorTest() {
	if (actionPreprocessing(true)) {
		uint8_t selectedBoardIndex;
		uint8_t selectedDeviceIndex;
		uint8_t maxInput;
		uint8_t lastBoardAndDeviceIndex;
		if (boardDisplayAndSelect(&selectedBoardIndex, &selectedDeviceIndex, &maxInput, &lastBoardAndDeviceIndex)) {
			print("\n\r");
			board[selectedBoardIndex]->oscillatorTest(selectedDeviceIndex);
		}
	}
}

/** Prints mrm-ref-can* calibration data
*/
void Robot::reflectanceArrayCalibrationPrint() {
	mrm_ref_can->calibrationDataRequest(0xFF, true);
	mrm_ref_can->calibrationPrint();
	actionEnd();
}

/** Starts robot's program
*/
void Robot::run() {
	while (true) {
		actionSet(); // Check if a key pressed and update current command buffer.
		if (_actionCurrent == NULL) // If last command finished, display menu.
			menu();
		else 
			actionProcess(); // Process current command. The command will be executed while currentCommand is not NULL. Here state maching processing occurs, too.
		noLoopWithoutThis(); // Receive all CAN Bus messages. This call should be included in any loop, like here.
	}
}

/** Reads serial ASCII input and converts it into an integer
@param timeoutFirst - timeout for first input
@param timeoutBetween - timeout between inputs
@param onlySingleDigitInput - completes input after first digit
@param limit - returns 0xFFFF if overstepped
@param printWarnings - prints out of range or timeout warnings
@return - converted number or 0xFFFF when timeout
*/
uint16_t Robot::serialReadNumber(uint16_t timeoutFirst, uint16_t timeoutBetween, bool onlySingleDigitInput, uint16_t limit, bool printWarnings) {

	// Read input
	uint32_t lastMs = millis();
	uint32_t convertedNumber = 0;
	bool any = false;
	while (millis() - lastMs < timeoutFirst && !any || !onlySingleDigitInput && millis() - lastMs < timeoutBetween && any) {
		if (Serial.available() || serialBT != NULL && serialBT->available()) {
			uint8_t character = 0;
			if (Serial.available())
				character = Serial.read();
			else if (serialBT != NULL && serialBT->available())
				character = serialBT->read();
			if (48 <= character && character <= 57) {
				convertedNumber = convertedNumber * 10 + (character - 48);
				any = true;
				lastMs = millis();
			}
		}
		noLoopWithoutThis();
	}

	// Eat tail
	while (Serial.available())
		Serial.read();
	while (serialBT != NULL && serialBT->available())
		serialBT->read();

	// Return result
	if (any) {
		if (convertedNumber > limit) {
			if (printWarnings)
				print("Out of range\n\r");
			return 0xFFFF;
		}
		else
			return convertedNumber;
	}
	else {
		if (printWarnings)
			print("Timeout.\n\r");
		return 0xFFFF;
	}
}

/** Moves servo motor manually
*/
void Robot::servoInteractive() {
	mrm_servo->writeInteractive();
	actionEnd();
}

/** Stops all motors
*/
void Robot::stopAll() {
	devicesStop();
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->count() > 0)
			((MotorBoard*)board[i])->stop();
	actionEnd();
}

/** CAN Bus stress test
*/
bool Robot::stressTest() {
	const bool STOP_ON_ERROR = true;
	const uint32_t LOOP_COUNT = 1000000;
	const bool TRY_ONLY_ALIVE = true;

	// Setup
	static uint32_t pass;
	static uint8_t lastPercent = 101;
	static uint8_t count[BOARDS_LIMIT];
	static uint32_t errors[BOARDS_LIMIT];
	static uint16_t mask[BOARDS_LIMIT]; // 16 bits - no more than 16 devices per board!

	if (actionPreprocessing(true)) {
		print("Before test.\n\r");
		pass = 0;
		devicesStop();
		for (uint8_t i = 0; i < _boardNextFree; i++)
			errors[i] = 0;
		uint8_t totalCnt = 0;
		for (uint8_t i = 0; i < _boardNextFree; i++) {
			count[i] = board[i]->devicesScan(true);
			totalCnt += count[i];
			mask[i] = TRY_ONLY_ALIVE ? 0 : 0xFFFF;
			for (uint8_t j = 0; j < board[i]->deadOrAliveCount(); j++)
				if (board[i]->alive(j))
					mask[i] |= 1 << j;
		}
		print("Start.\n\r");
		if (mrm_8x8a->alive()) {
			char buffer[50];
			sprintf(buffer, "%i devices.\n\r", totalCnt);
			mrm_8x8a->text(buffer);
		}
	}

	// Display progress numerically
	uint8_t percent = 100 * pass / LOOP_COUNT;
	if (percent != lastPercent && percent > 0) {
		lastPercent = percent;
		print("%i %%\n\r", percent);
	}

	// Display progress using mrm-8x8a
	if (mrm_8x8a->alive())
		if (mrm_8x8a->progressBar(LOOP_COUNT, pass, pass == 0))
			delayMs(5); // To avoid disturbing stress test

	// Stress test
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		if (count[i] > 0 || !TRY_ONLY_ALIVE) {
			delayMicros(40);
			digitalWrite(15, HIGH);
			uint8_t cnt = board[i]->devicesScan(false, mask[i]);
			digitalWrite(15, LOW);
			if (cnt != count[i]) {
				errors[i]++;
				print("***** %s: found %i, not %i.\n\r", board[i]->name(), cnt, count[i]);
				if (STOP_ON_ERROR) {
					if (mrm_8x8a->alive()) {
						char buffer[50];
						sprintf(buffer, "%s: error.\n\r", board[i]->name());
						mrm_8x8a->text(buffer);
					}
					pass = LOOP_COUNT - 1;
					break;
				}
			}
		}
	}

	// Results
	if (++pass >= LOOP_COUNT || userBreak()) {
		bool allOK = true;
		for (uint8_t i = 0; i < _boardNextFree; i++)
			if (count[i] > 0 && errors[i] > 0) {
				print("%s: %i errors.\n\r", board[i]->name(), errors[i]);
				allOK = false;
				delay(5000); // To freeze oscilloscope
			}
		if (allOK) {
			if (mrm_8x8a->alive())
				mrm_8x8a->bitmapDisplay(0);
			print("No errors.");
		}
		print("\n\r");
		actionEnd();
		return true;
	}
	else
		return false;
}

/** Tests mrm-therm-b-can
*/
void Robot::thermoTest() {
	if (actionPreprocessing(true))
		mrm_therm_b_can->start();
	mrm_therm_b_can->test();
}

/** Checks if user tries to break the program
@return - true if break requested.
*/
bool Robot::userBreak() {
	if (/*switchOn() ||*/ Serial.available() || serialBT != NULL && serialBT->available()) {
		return true;
	}
	else
		return false;
}

/** Prints additional data in every loop pass
*/
void Robot::verbosePrint() {
	if (verbose) {
		static uint32_t lastMs = 0;
		if (lastMs == 0 || millis() - lastMs > 5000) {
			print("%i fps\r\n", (uint16_t)fpsGet());
			lastMs = millis();
		}
	}
}

/** Verbose output toggle
*/
void Robot::verboseToggle() {
	verbose = !verbose;
};

///** Print to all serial ports, pointer to list
//*/
//void Robot::vprint(const char* fmt, va_list argp) {
//	if (strlen(fmt) >= 100)
//		return;
//	static char buffer[100];
//	vsprintf(buffer, fmt, argp);
//
//	Serial.print(buffer);
//	if (serialBT() != 0)
//		serialBT()->print(buffer);
//}
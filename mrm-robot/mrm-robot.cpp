#include <mrm-board.h>
#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-col-can.h>
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
#include <mrm-robot.h>
#include <mrm-servo.h>
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
#include <mrm-us.h>

/**
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Robot::Robot(BluetoothSerial* hardwareSerial) {
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
	Serial.begin(115200);
	serial = new BluetoothSerial();
	serial->begin("ESP32"); //Start Bluetooth. ESP32 - Bluetooth device name, choose one.
	delay(50);
	print("Robot started.\r\n");

	Wire.begin(); // Start I2C

	esp32CANBus = new ESP32CANBus();

	_actionCurrent = NULL;
	_actionPrevious = _actionCurrent;

	actionAdd(new Action8x8Test(this));
	actionAdd(new ActionAny(this));
	actionAdd(new ActionBluetoothTest(this));
	actionAdd(new ActionCANBusScan(this));
	actionAdd(new ActionCANBusSniff(this));
	actionAdd(new ActionCANBusStress(this));
	actionAdd(new ActionColorTest(this));
	actionAdd(new ActionDeviceIdChange(this));
	actionAdd(new ActionFirmware(this));
	actionAdd(new ActionFPS(this));
	actionAdd(new ActionGoAhead(this));
	actionAdd(new ActionI2CTest(this));
	actionAdd(new ActionIMUTest(this));
	actionAdd(new ActionIRFinderTest(this));
	actionAdd(new ActionIRFinderCanTest(this));
	actionAdd(new ActionIRFinderCanTestCalculated(this));
	actionAdd(new ActionLidar2mTest(this));
	actionAdd(new ActionLidar4mTest(this));
	actionAdd(new ActionLidarCalibrate(this));
	actionAdd(new ActionMenuMain(this));
	actionAdd(new ActionMotorTest(this));
	actionAdd(new ActionNodeTest(this));
	actionAdd(new ActionNodeServoTest(this));
	actionAdd(new ActionReflectanceArrayCalibrate(this));
	actionAdd(new ActionReflectanceArrayCalibrationPrint(this));
	actionAdd(new ActionReflectanceArrayAnalogTest(this));
	actionAdd(new ActionReflectanceArrayDigitalTest(this));
	actionAdd(new ActionServoTest(this));
	actionAdd(new ActionStop(this));
	actionAdd(new ActionThermoTest(this));

	_actionAny = new ActionAny(this);
	_actionDoNothing = new ActionDoNothing(this);
	_actionStop = new ActionStop(this);

	mrm_8x8a = new Mrm_8x8a(this);
	mrm_bldc2x50 = new Mrm_bldc2x50(this);
	mrm_bldc4x2_5 = new Mrm_bldc4x2_5(this);
	mrm_col_can = new Mrm_col_can(this);
	mrm_imu = new Mrm_imu(this);
	mrm_ir_finder2 = new Mrm_ir_finder2(this);
	mrm_ir_finder_can = new Mrm_ir_finder_can(this);
	mrm_lid_can_b = new Mrm_lid_can_b(this, 10);
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

	mrm_switch->actionSet(_actionAny, 0);
	mrm_switch->actionSet(_actionStop, 1);
	
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

	// IMU
	mrm_imu->add();

	// mrm-ir-finder2
	mrm_ir_finder2->add(34, 33);

	// mrm-ir-finder-can
	mrm_ir_finder_can->add("IRFind-0");

	// Motors mrm-mot2x50
	mrm_mot2x50->add(false, "Mot2x50-0");
	mrm_mot2x50->add(false, "Mot2x50-1");
	mrm_mot2x50->add(false, "Mot2x50-2");
	mrm_mot2x50->add(false, "Mot2x50-3");
	mrm_mot2x50->add(false, "Mot2x50-4");
	mrm_mot2x50->add(false, "Mot2x50-5");

	// Motors mrm-mot4x10
	mrm_mot4x10->add(true, "Mot4x10-0");
	mrm_mot4x10->add(true, "Mot4x10-1");
	mrm_mot4x10->add(true, "Mot4x10-2");
	mrm_mot4x10->add(true, "Mot4x10-3");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can->add(false, "Mot3.6-0");
	mrm_mot4x3_6can->add(false, "Mot3.6-1");
	mrm_mot4x3_6can->add(true, "Mot3.6-2");
	mrm_mot4x3_6can->add(true, "Mot3.6-3");

	mrm_mot4x3_6can->add(false, "Mot3.6-4");
	mrm_mot4x3_6can->add(false, "Mot3.6-5");
	mrm_mot4x3_6can->add(true, "Mot3.6-6");
	mrm_mot4x3_6can->add(true, "Mot3.6-7");

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

	// Servo motors
	mrm_servo->add(16, "Servo", 10);

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
	add(mrm_ir_finder_can);
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
	if (nextFreeAction >= COMMANDS_LIMIT) {
		strcpy(errorMessage, "COMMANDS_LIMIT exceeded.");
		return;
	}
	_action[nextFreeAction++] = action;
}

/** Actually perform the action
*/
void Robot::actionProcess() {
	if (_actionCurrent != NULL) {
		_actionCurrent->perform();
		//if (_actionCurrent != NULL)
		//	_actionCurrent->_firstProcess = false;
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
	if (mrm_8x8a->actionCheck() != NULL)
		_actionCurrent = mrm_8x8a->actionCheck();
	else if (mrm_switch->actionCheck())
		_actionCurrent = mrm_8x8a->actionCheck();
	else { // Check keyboard
		if (Serial.available() || serial->available()) {
			lastUserActionMs = millis();
			uint8_t ch;
			if (Serial.available())
				ch = Serial.read();
			else
				ch = serial->read();

			if (ch != 13) //if received data different from ascii 13 (enter)
				uartRxCommandCumulative[uartRxCommandIndex++] = ch;	//add data to Rx_Buffer

			if (ch == 13 || uartRxCommandIndex >= 3 || ch == 'x') //if received data = 13
			{
				uartRxCommandCumulative[uartRxCommandIndex] = 0;
				uartRxCommandIndex = 0;

				print("Command: %s", uartRxCommandCumulative);

				uint8_t found = 0;
				for (uint8_t i = 0; i < nextFreeAction; i++) {
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
	_actionCurrent->initializationSet(true);
}

/** Add a new board to the collection of possible boards for the robot
@param aBoard - the board.
*/
void Robot::add(Board* aBoard) {
	if (nextFreeBoardSlot > MRM_BOARD_COUNT - 1) {
		strcpy(errorMessage, "Too many boards");
		return;
	}
	board[nextFreeBoardSlot++] = aBoard;
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
void Robot::canBusSniff() {
	if (esp32CANBus->messageReceive()) {
		bool found = mrm_8x8a->messagePrint(esp32CANBus->rx_frame);
		print("\n\r");
	}
}

/** Change device's id
*/
void Robot::canIdChange() {
	// Print all devices alive
	uint8_t last = 0;
	for (uint8_t boardNumber = 0; boardNumber < nextFreeBoardSlot; boardNumber++) {
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
	else {

		// Choose device to be changed
		print("Enter board [1 - %i]: ", last);
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

			// Find selected board
			last = 0;
			Board* selectedBoard = NULL;
			uint8_t selectedDeviceIndex = 0xFF;
			uint8_t maxInput = 0;
			for (uint8_t boardNumber = 0; boardNumber < nextFreeBoardSlot && selectedDeviceIndex == 0xFF; boardNumber++) {
				uint8_t currentCount = 0;
				for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount() && selectedDeviceIndex == 0xFF; deviceNumber++)
					if (board[boardNumber]->alive(deviceNumber)) {
						if (currentCount == 0) {
							if (++last == selectedNumber) {
								selectedBoard = board[boardNumber];
								selectedDeviceIndex = deviceNumber;
								maxInput = board[boardNumber]->deadOrAliveCount() / board[boardNumber]->devicesOnASingleBoard() - 1;
								break;
							}
						}

						if (++currentCount == board[boardNumber]->devicesOnASingleBoard())
							currentCount = 0;
					}
			}

			// Enter new id
			print("%i. %s\n\rEnter new board id [0..%i]: ", last, selectedBoard->name(), maxInput);
			lastMs = millis();
			uint8_t newDeviceNumber = 0;
			bool any = false;
			while ((millis() - lastMs < 30000 && !any || maxInput > 9 && millis() - lastMs < 500 && any) && newDeviceNumber < maxInput)
				if (Serial.available()) {
					newDeviceNumber = newDeviceNumber * 10 + (Serial.read() - 48);
					any = true;
					lastMs = millis();
				}

			if (newDeviceNumber > maxInput)
				print("timeout\n\r");
			else {

				// Change
				print("%i\n\rChange requested.\n\r", newDeviceNumber);
				selectedBoard->idChange(newDeviceNumber, selectedDeviceIndex);
				delay(500); // Delay for firmware handling of devices with the same ids.
			}
		}
	}
	actionEnd();
}

/** mrm-color-can test
*/
void Robot::colorTest() {
	if (actionInitialization(true))
		mrm_col_can->start();
	mrm_col_can->test();
}

/** The right way to use Arduino function delay
@param pauseMs - pause in ms.
*/
void Robot::delayMs(uint16_t pauseMs) {
	uint32_t startMs = millis();
	while (millis() - startMs < pauseMs)
		noLoopWithoutThis();
}

/** Contacts all the CAN Bus devices and checks which one is alive.
@verbose - if true, print. 
*/
void Robot::devicesScan(bool verbose) {
	devicesStop();
	delayMs(100); // Read all the messages sent after stop.
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
		board[i]->devicesScan(verbose);
	actionEnd();
}

/** Starts devices' CAN Bus messages broadcasting.
*/
void Robot::devicesStart(uint8_t measuringMode) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFreeBoardSlot; deviceNumber++)
		board[deviceNumber]->start(0xFF, measuringMode);
}

/** Stops broadcasting of CAN Bus messages
*/
void Robot::devicesStop() {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFreeBoardSlot; deviceNumber++) {
		board[deviceNumber]->stop();
		delayMs(1); // TODO
	}
}

/** Displays errors and stops motors, if any.
*/
void Robot::errors() {
	if (strcmp(errorMessage, "") != 0) {
		print("ERROR! %s\n\r", errorMessage);
		stopAll(); // Stop all motors
		actionEnd();
		strcpy(errorMessage, "");
	}
}

/** Displays each CAN Bus device's firmware
*/
void Robot::firmwarePrint() {
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
		board[i]->firmwareRequest();
		uint32_t startMs = millis();
		while (millis() - startMs < 10)
			noLoopWithoutThis();
		board[i]->firmwareDisplay();
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
		fpsNow = 1000;
	else
		fpsNow = 1000.0 / (float)(fpsMs[1] > fpsMs[0] ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
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
	print("CAN peaks: %i received/s, %i sent/s\n\r", esp32CANBus->messagesPeakReceived(), esp32CANBus->messagesPeakSent());
	print("Arduino: %i FPS, low peak: %i FPS\n\r", (int)fpsGet(), fpsTopGap == 1000 ? 0 : (int)(1000 / (float)fpsTopGap));
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
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
}

/** Updates data for FPS calculation
*/
void Robot::fpsUpdate() {
	fpsMs[fpsNextIndex] = millis();
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

/** Tests mrm-ir-finder-can, raw data.
*/
void Robot::irFinderCanTest() {
	if (actionInitialization(true))
		mrm_ir_finder_can->start();
	mrm_ir_finder_can->test();
}

/** Tests mrm-ir-finder-can, calculated data.
*/
void Robot::irFinderCanTestCalculated() {
	if (actionInitialization(true))
		mrm_ir_finder_can->continuousReadingCalculatedDataStart();
	mrm_ir_finder_can->testCalculated();
}

/** Tests mrm-lid-can-b
*/
void Robot::lidar2mTest() {
	if (actionInitialization(true))
		mrm_lid_can_b->start();
	mrm_lid_can_b->test();
}

/** Tests mrm-lid-can-b2
*/
void Robot::lidar4mTest() {
	if (actionInitialization(true))
		mrm_lid_can_b2->start();
	mrm_lid_can_b2->test();
}

/** Calibrates lidars
*/
void Robot::lidarCalibrate() {
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
					mrm_lid_can_b->calibration(lidarNumber - 1);
				else
					mrm_lid_can_b2->calibration(lidarNumber - 1);
			}
		}
	}

	actionEnd();
}

/** Displays menu
*/
void Robot::menu() {
	// Print menu
	devicesScan(false);
	print("\r\n");

	bool any = false;
	uint8_t column = 1;
	uint8_t maxColumns = 2;

	for (uint8_t i = 0; i < nextFreeAction; i++) {
		if ((_action[i]->_menuLevel | menuLevel) == _action[i]->_menuLevel) {
			print("%-3s - %-22s%s", _action[i]->_shortcut, _action[i]->_text, column == maxColumns ? "\n\r" : "");
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
	for (uint8_t deviceNumber = 0; deviceNumber < nextFreeBoardSlot; deviceNumber++)
		if (board[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", board[deviceNumber]->errorCodeLast(), board[deviceNumber]->name(board[deviceNumber]->errorWasInDeviceNumber()));

	fpsPause(); // this function took too much time

	_actionCurrent = _actionDoNothing;
}

/** Displays menu and stops motors
*/
void Robot::menuMainAndIdle() {
	stopAll();
	menuLevel = 1;
}

/** Receives CAN Bus messages. 
*/
void Robot::messagesReceive() {
	while (esp32CANBus->messageReceive()) {
		uint32_t id = esp32CANBus->rx_frame->MsgID;
		bool any = false;
		for (uint8_t deviceGroupNumber = 0; deviceGroupNumber < nextFreeBoardSlot; deviceGroupNumber++) {
			if (board[deviceGroupNumber]->messageDecode(id, esp32CANBus->rx_frame->data.u8)) {
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
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) 
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->aliveCount() > 0)
			board[i]->test();
	actionEnd();
}

/** Tests mrm-node
*/
void Robot::nodeTest() {
	if (actionInitialization(true))
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

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Robot::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Prints mrm-ref-can* calibration data
*/
void Robot::reflectanceArrayCalibrationPrint() {
	mrm_ref_can->calibrationDataRequest(0xFF, true);
	mrm_ref_can->calibrationPrint();
	actionEnd();
}

/** Tests mrm-ref-can*
@digital - digital data. Otherwise analog.
*/
void Robot::reflectanceArrayTest(bool digital) {
	if (actionInitialization(true))
		mrm_ref_can->start(0xFF, digital ? 1 : 0);
	mrm_ref_can->test();
	fpsPause();
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

/** Stops all motors
*/
void Robot::stopAll() {
	devicesStop();
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
		if (board[i]->boardType() == MOTOR_BOARD && board[i]->aliveCount() > 0)
			((MotorBoard*)board[i])->stop();
	actionEnd();
}

/** CAN Bus stress test
*/
bool Robot::stressTest() {
	const bool STOP_ON_ERROR = false;
	const uint16_t LOOP_COUNT = 10000;

	// Setup
	static uint32_t pass;
	static uint8_t lastPercent = 101;
	static uint8_t count[MRM_BOARD_COUNT];
	static uint32_t errors[MRM_BOARD_COUNT];
	if (actionInitialization(true)) {
		print("Before test.\n\r");
		pass = 0;
		devicesStop();
		for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
			errors[i] = 0;
		for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
			delay(1);
			count[i] = board[i]->devicesScan(true);
		}
		print("Start.\n\r");
	}

	// Stress test
	uint8_t percent = 100 * pass / LOOP_COUNT;
	if (percent != lastPercent) {
		lastPercent = percent;
		print("%i %%\n\r", percent);
	}
	for (uint8_t i = 0; i < nextFreeBoardSlot; i++) {
		if (count[i] > 0) {
			delay(1);
			uint8_t cnt = board[i]->devicesScan(false);
			if (cnt != count[i]) {
				errors[i]++;
				print("***** %s: found %i, not %i.\n\r", board[i]->name(), cnt, count[i]);
				if (STOP_ON_ERROR)
					break;
			}
		}
	}

	// Results
	if (++pass >= LOOP_COUNT) {
		bool allOK = true;
		for (uint8_t i = 0; i < nextFreeBoardSlot; i++)
			if (count[i] > 0 && errors[i] > 0) {
				print("%s: %i errors.\n\r", board[i]->name(), errors[i]);
				allOK = false;
			}
		if (allOK)
			print("No errors.");
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
	if (actionInitialization(true))
		mrm_therm_b_can->start();
	mrm_therm_b_can->test();
}

/** Checks if user tries to break the program
@return - true if break requested.
*/
bool Robot::userBreak() {
	if (/*switchOn() ||*/ Serial.available() || serial->available()) {
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

/** Print to all serial ports, pointer to list
*/
void Robot::vprint(const char* fmt, va_list argp) {
	if (strlen(fmt) >= 100)
		return;
	static char buffer[100];
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serialBT() != 0)
		serialBT()->print(buffer);
}
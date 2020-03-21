#pragma once
#include <mrm-action.h>

#define ACTIONS_LIMIT 50 // Increase if more actions are needed.
#define BOARDS_LIMIT 14 // Maximum number of different board types.
#define LED_ERROR 15 // mrm-esp32's pin number, hardware defined.
#define LED_OK 2 // mrm-esp32's pin number, hardware defined.

// Forward declarations

class Mrm_8x8a;
class Mrm_bldc2x50;
class Mrm_bldc4x2_5;
class Mrm_col_can;
class Mrm_imu;
class Mrm_ir_finder2;
class Mrm_ir_finder_can;
class Mrm_lid_can_b;
class Mrm_lid_can_b2;
class Mrm_mot2x50;
class Mrm_mot4x10;
class Mrm_mot4x3_6can;
class Mrm_node;
class Mrm_ref_can;
class Mrm_servo;
class Mrm_switch;
class Mrm_therm_b_can;
class Mrm_us;

/** Base class for all robots.
*/
class Robot {

protected:

	ActionBase* _action[ACTIONS_LIMIT]; // Collection of all the robot's actions
	uint8_t _actionNextFree = 0;

	// Robot's actions that can be callect directly, not just by iterating _action collection
	ActionBase* _actionAny;
	ActionBase* _actionCurrent;
	ActionBase* _actionDoNothing;
	ActionBase* _actionStop;
	ActionBase* _actionPrevious;

	Board* board[BOARDS_LIMIT]; // Collection of all the robot's boards
	uint8_t _boardNextFree = 0;

	// FPS - frames per second calculation
	uint32_t fpsMs[2] = { 0, 0 };
	uint8_t fpsNextIndex = 0;
	uint32_t fpsTopGap = 0;

	uint8_t menuLevel = 1; // Submenus have bigger numbers
	BluetoothSerial* serial; // Additional serial port
	bool verbose = false; // Verbose output

	/** Actually perform the action
	*/
	void actionProcess();

	/** User sets a new action, using keyboard or Bluetooth
	*/
	void actionSet();

	/** New action is set in the program
	@param newAction - the new action.
	*/
	void actionSet(ActionBase* newAction);

	/** Avoids FPS measuring in the next 2 cycles.
	*/
	void fpsPause();

	/** Updates data for FPS calculation
	*/
	void fpsUpdate();

	/** Resets FPS data
	*/
	void fpsReset();

	/** Prints additional data in every loop pass
	*/
	void verbosePrint();

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

public:

	char errorMessage[60] = ""; // Global variable enables functions to set it although not passed as parameter
	ESP32CANBus* esp32CANBus; // CANBus interface

	Mrm_8x8a* mrm_8x8a;
	Mrm_bldc2x50* mrm_bldc2x50;
	Mrm_bldc4x2_5* mrm_bldc4x2_5;
	Mrm_col_can* mrm_col_can;
	Mrm_imu* mrm_imu;
	Mrm_ir_finder2* mrm_ir_finder2;
	Mrm_ir_finder_can* mrm_ir_finder_can;
	Mrm_lid_can_b* mrm_lid_can_b;// 10
	Mrm_lid_can_b2* mrm_lid_can_b2;
	Mrm_mot2x50* mrm_mot2x50;
	Mrm_mot4x3_6can* mrm_mot4x3_6can;
	Mrm_mot4x10* mrm_mot4x10;
	Mrm_node* mrm_node;
	Mrm_ref_can* mrm_ref_can;
	Mrm_servo* mrm_servo;
	Mrm_switch* mrm_switch;
	Mrm_therm_b_can* mrm_therm_b_can;
	Mrm_us* mrm_us;

	/**
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Robot(BluetoothSerial* hardwareSerial = 0);

	/** Add a new action to the collection of robot's possible actions.
	@param action - the new action.
	*/
	void actionAdd(ActionBase* action);

	/** End current action
	*/
	void actionEnd() { _actionCurrent = NULL; }

	/** Is this current action's initialization
	@param andFinish - finish initialization
	@return - it is.
	*/
	bool actionPreprocessing(bool andFinish) { 
		bool itIs = _actionCurrent->preprocessing(); 
		if (andFinish)
			_actionCurrent->preprocessingEnd();
		return itIs; 
	}

	/** Finish action's intialization phase
	*/
	void actionPreprocessingEnd() { _actionCurrent->preprocessingEnd(); }

	/** Add a new board to the collection of possible boards for the robot
	@param aBoard - the board.
	*/
	void add(Board* aBoard);

	/** User test, defined in derived classes.
	*/
	virtual void anyTest() = 0;

	/** Store bitmaps in mrm-led8x8a.
	*/
	virtual void bitmapsSet() = 0;

	/** Blink LED
	*/
	void blink();

	/** Test Bluetooth
	*/
	void bluetoothTest();

	/** Display all the incomming and outcomming CAN Bus messages
	*/
	void canBusSniff();

	/** Change device's id
	*/
	void canIdChange();

	/** mrm-color-can test
	*/
	void colorTest();

	/** The right way to use Arduino function delay
	@param pauseMs - pause in ms. One run even if pauseMs == 0, so that delayMs(0) receives all messages.
	*/
	void delayMs(uint16_t pauseMs);

	/** Contacts all the CAN Bus devices and checks which one is alive.
	@verbose - if true, print.
	*/
	void devicesScan(bool verbose);

	/** Starts devices' CAN Bus messages broadcasting.
	*/
	void devicesStart(uint8_t measuringMode = 0);

	/** Stops broadcasting of CAN Bus messages
	*/
	void devicesStop();

	/** Displays errors and stops motors, if any.
	*/
	void errors();

	/** Displays each CAN Bus device's firmware
	*/
	void firmwarePrint();

	/** Returns FPS (frames per second).
	@return - FPS
	*/
	float fpsGet();

	/** Prints FPS all CAN Bus devices and mrm-eps32 boards. Also prints CAN Bus frequency.
	*/
	void fpsPrint();

	/** Orders the robot to go ahead
	*/
	virtual void goAhead() = 0;

	/** Lists I2C devices
	*/
	void i2cTest();

	/** Tests mrm-ir-finder-can, raw data.
	*/
	void irFinderCanTest();

	/** Tests mrm-ir-finder-can, calculated data.
	*/
	void irFinderCanTestCalculated();

	/** Tests mrm-lid-can-b
	*/
	void lidar2mTest();

	/** Tests mrm-lid-can-b2
	*/
	void lidar4mTest();

	/** Calibrates lidars
	*/
	void lidarCalibrate();

	/** Displays menu
	*/
	void menu();

	/** Displays menu and stops motors
	*/
	void menuMainAndIdle();

	/** Receives CAN Bus messages.
	*/
	void messagesReceive();

	/** Tests motors
	*/
	void motorTest();

	/** Tests mrm-node
	*/
	void nodeTest();

	/** Any for or while loop must include call to this function.
*/
	void noLoopWithoutThis();

	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	/** Prints mrm-ref-can* calibration data
	*/
	void reflectanceArrayCalibrationPrint();

	/** Tests mrm-ref-can*
	@digital - digital data. Otherwise analog.
	*/
	void reflectanceArrayTest(bool digital = true);

	/** Starts robot's program
	*/
	void run();

	/** Reads serial ASCII input and converts it into an integer
	@param timeoutFirst - timeout for first input
	@param timeoutBetween - timeout between inputs
	@param onlySingleDigitInput - completes input after first digit
	@param limit - returns 0xFFFF if overstepped
	@param printWarnings - prints out of range or timeout warnings
	@return - converted number or 0xFFFF when timeout
	*/
	uint16_t serialReadNumber(uint16_t timeoutFirst = 3000, uint16_t timeoutBetween = 500, bool onlySingleDigitInput = false, uint16_t limit = 0xFFFE, bool printWarnings = true);

	/** Bluetooth
	@return - Bluetooth object
	*/
	BluetoothSerial* serialBT() { return serial; }

	/** Stops all motors
	*/
	void stopAll();

	/** CAN Bus stress test
	*/
	bool stressTest();

	/** Tests mrm-therm-b-can
	*/
	void thermoTest();

	/** Checks if user tries to break the program
	@return - true if break requested.
	*/
	bool userBreak();

	/** Verbose output toggle
	*/
	void verboseToggle();
};
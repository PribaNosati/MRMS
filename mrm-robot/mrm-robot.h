#pragma once
#include <mrm-action.h>

#define COMMANDS_LIMIT 50 // Increase if more commands are needed
#define LED_ERROR 15 // Pin number, hardware defined
#define LED_OK 2 // Pin number, hardware defined
#define MRM_BOARD_COUNT 14

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

class Robot {

protected:
	uint8_t fpsNextIndex = 0; // To count frames per second
	uint32_t fpsMs[3] = { 0, 0, 0 };
	uint8_t nextFreeAction = 0;
	uint8_t nextFreeCommand = 0;
	uint8_t nextFreeBoardSlot = 0;
	BluetoothSerial* serial; // Additional serial port
	bool verbose = false; // Verbose output

	void fps();

	/** Print to all serial ports, pointer to list
	*/
	void vprint(const char* fmt, va_list argp);

public:
	ActionBase* _actionAny;
	ActionBase* _actionDoNothing;
	ActionBase* _actionStop;

	ActionBase* _action[COMMANDS_LIMIT];
	ActionBase* _actionCurrent;
	ActionBase* _actionPrevious;

	Board* board[MRM_BOARD_COUNT];
	ESP32CANBus* esp32CANBus; // CANBus interface
	char errorMessage[60] = ""; // Global variable enables functions to set it although not passed as parameter
	uint8_t menuLevel = 1; // Submenus have bigger numbers

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

	void actionAdd(ActionBase* action);

	void actionProcess();

	void actionSet(ActionBase* newAction);

	void actionUpdate();

	void add(Board* aBoard);

	void anyTest();

	void blink();

	void bluetoothTest();

	void broadcastingStart(uint8_t measuringMode = 0);

	void broadcastingStop();

	uint8_t boardCount() { return nextFreeBoardSlot; }

	void canBusSniff();

	void canIdChange();

	void colorTest();

	void devicesScan(bool verbose);

	void errors();

	void firmwarePrint();

	void fpsPrint();

	virtual void goAhead() = 0;

	void i2cTest();

	void irFinderCanTest();

	void irFinderCanTestCalculated();

	void lidar2mTest();

	void lidar4mTest();

	void lidarCalibrate();

	void menu();

	void menuMainAndIdle();

	void messagesReceive();

	void motorTest();

	void nodeTest();

	void noLoopWithoutThis();

	/** Print to all serial ports
	@param fmt - C format string
	@param ... - variable arguments
	*/
	void print(const char* fmt, ...);

	void reflectanceArrayCalibrationPrint();

	void reflectanceArrayTest();

	void run();

	BluetoothSerial* serialBT() { return serial; }

	void stopAll();

	bool stressTest();

	void thermoTest();

	bool userBreak();

	void verbosePrint();

	void verboseToggle();
};
#pragma once
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <mrm-board.h>
#include <mrm-ref-can.h>

class Robot;
class ActionBase {
protected:
	bool _preprocessing = true;
	Robot* _robot;

public:
	char _shortcut[4];
	char _text[20];
	uint8_t _menuLevel;

	ActionBase(Robot* robot, char shortcut[4], char text[20], uint8_t menuLevel = 1);

	bool preprocessing() { return _preprocessing; }

	bool preprocessingEnd() { _preprocessing = false; }

	bool preprocessingStart() { _preprocessing = true; }

	virtual void performAfter() {}; // todo, not implemented.

	virtual void performBefore() {};

	virtual void perform() = 0;
};

class Action8x8Test : public ActionBase {
	void perform();
public:
	Action8x8Test(Robot* robot) : ActionBase(robot, "led", "Test 8x8", 1) {}
};

class ActionAny : public ActionBase {
	void perform();
public:
	ActionAny(Robot* robot) : ActionBase(robot, "any", "Any test", 1) {}
};

class ActionBluetoothTest : public ActionBase {
	void perform();
public:
	ActionBluetoothTest(Robot* robot) : ActionBase(robot, "blt", "Test Bluetooth", 1) {}
};

class ActionCANBusScan : public ActionBase {
	void perform();
public:
	ActionCANBusScan(Robot* robot) : ActionBase(robot, "can", "Report devices", 1) {}
};

class ActionCANBusSniff : public ActionBase {
	void perform();
public:
	ActionCANBusSniff(Robot* robot) : ActionBase(robot, "sni", "Sniff CAN Bus", 1) {}
};

class ActionCANBusStress : public ActionBase {
	void perform();
public:
	ActionCANBusStress(Robot* robot) : ActionBase(robot, "all", "CAN Bus stress", 1) {}//2 | 4 | 8 | 16 -> in all menus. 0 - in no menu.
};

class ActionColorTest : public ActionBase {
	void perform();
public:
	ActionColorTest(Robot* robot) : ActionBase(robot, "col", "Test color", 1) {}
};

class ActionDeviceIdChange : public ActionBase {
	void perform();
public:
	ActionDeviceIdChange(Robot* robot) : ActionBase(robot, "idc", "Device's id change", 1) {}
};

class ActionDoNothing : public ActionBase {
	void perform() {}
public:
	ActionDoNothing(Robot* robot) : ActionBase(robot, "", "No action", 0) {}
};

class ActionFirmware : public ActionBase {
	void perform();
public:
	ActionFirmware(Robot* robot) : ActionBase(robot, "fir", "Firmware", 1) {}
};

class ActionFPS : public ActionBase {
	void perform();
public:
	ActionFPS(Robot* robot) : ActionBase(robot, "fps", "FPS", 1) {}
};

class ActionGoAhead : public ActionBase {
	void perform();
public:
	ActionGoAhead(Robot* robot) : ActionBase(robot, "ahe", "Go ahead", 1) {}
};

class ActionI2CTest : public ActionBase {
	void perform();
public:
	ActionI2CTest(Robot* robot) : ActionBase(robot, "i2c", "Test I2C", 1) {}
};

class ActionIMUTest : public ActionBase {
	void perform();
public:
	ActionIMUTest(Robot* robot) : ActionBase(robot, "imu", "Test IMU", 1) {}
};

class ActionIRFinderTest : public ActionBase {
	void perform();
public:
	ActionIRFinderTest(Robot* robot) : ActionBase(robot, "irf", "Test ball analog", 1) {}
};

class ActionIRFinderCanTest : public ActionBase {
	void perform();
public:
	ActionIRFinderCanTest(Robot* robot) : ActionBase(robot, "irs", "Test ball CAN sing.", 1) {}
};

class ActionIRFinderCanTestCalculated : public ActionBase {
	void perform();
public:
	ActionIRFinderCanTestCalculated(Robot* robot) : ActionBase(robot, "irc", "Test ball CAN calc.", 1) {}
};

class ActionLidar2mTest : public ActionBase {
	void perform();
public:
	ActionLidar2mTest(Robot* robot) : ActionBase(robot, "li2", "Test li. 2m", 1) {}
};

class ActionLidar4mTest : public ActionBase {
	void perform();
public:
	ActionLidar4mTest(Robot* robot) : ActionBase(robot, "li4", "Test li. 4m", 1){}
};

class ActionLidarCalibrate : public ActionBase {
	void perform();
public:
	ActionLidarCalibrate(Robot* robot) : ActionBase(robot, "lic", "Cal. lidar", 1) {}
};

class ActionMenuMain : public ActionBase {
	void perform();
public:
	ActionMenuMain(Robot* robot) : ActionBase(robot, "x", "Escape", 2 | 4 | 8 | 16) {}//2 | 4 | 8 | 16 -> in all menus. 0 - in no menu.
};

class ActionMotorTest : public ActionBase {
	void perform();
public:
	ActionMotorTest(Robot* robot) : ActionBase(robot, "mot", "Test motors", 1) {}
};

class ActionNodeTest : public ActionBase {
	void perform();
public:
	ActionNodeTest(Robot* robot) : ActionBase(robot, "nod", "Test node", 1) {}
};

class ActionNodeServoTest : public ActionBase {
	void perform();
public:
	ActionNodeServoTest(Robot* robot) : ActionBase(robot, "nos", "Test node servo", 1) {}
};

class ActionStop : public ActionBase {
	void perform();
public:
	ActionStop(Robot* robot) : ActionBase(robot, "sto", "Stop", 1) {}
};

class ActionReflectanceArrayAnalogTest : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayAnalogTest(Robot* robot) : ActionBase(robot, "ref", "Test refl. anal.", 1) {}
};

class ActionReflectanceArrayDigitalTest : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayDigitalTest(Robot* robot) : ActionBase(robot, "red", "Test refl. digi.", 1) {}
};

class ActionReflectanceArrayCalibrate : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayCalibrate(Robot* robot) : ActionBase(robot, "cal", "Calibrate refl.", 1) {}
};

class ActionReflectanceArrayCalibrationPrint : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayCalibrationPrint(Robot* robot) : ActionBase(robot, "cpr", "Calibration print", 1) {}
};

class ActionServoTest : public ActionBase {
	void perform();
public:
	ActionServoTest(Robot* robot) : ActionBase(robot, "ser", "Test servo", 1) {}
};

class ActionThermoTest : public ActionBase {
	void perform();
public:
	ActionThermoTest(Robot* robot) : ActionBase(robot, "the", "Test thermo", 1) {}
};

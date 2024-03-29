#pragma once
#include <Arduino.h>
#include <mrm-board.h>
#include <mrm-ref-can.h>

struct LEDSign{
	uint8_t type;
};

struct LEDSignBitmap : LEDSign{
	uint8_t red[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t green[8] =  {0, 0, 0, 0, 0, 0, 0, 0};;
	LEDSignBitmap(){type = 0;}
};

struct LEDSignText : LEDSign{
	char text[8];
	LEDSignText(){type = 1;}
};

class Robot;
class ActionBase {
protected:
	BoardId _boardsId;
	bool _preprocessing = true;
	Robot* _robot;

public:
	char _shortcut[4];
	char _text[19];
	uint8_t _menuLevel;
	LEDSign* ledSign;

	/** Constructor
	@param robot - robot
	@param shortcut - up-to-3-letter word
	@param text - menu entry
	@param menuLevel - all the actions with the same menuLevel are displayed. Permitted values are: 0 (in no menu), 1, 2, 4, 16, 16, 32, 64, and 128. 
		A menu-action changes menuLevel to its own, forcing all the actions with this menuLevel to be displayed. "|" lists action in many menus, for example 1 | 16 | 16.
	@param boardId - menu only for a specific board
	@param ledSign - the LED sign that will be displayed when action set to this one
	*/
	ActionBase(Robot* robot, const char shortcut[4], const char text[20], uint8_t menuLevel = 1, BoardId boardsId = ID_ANY,
		LEDSign* ledSign8x8 = NULL);

	BoardId boardsId() { return _boardsId; }

	bool preprocessing() { return _preprocessing; }

	void preprocessingEnd() { _preprocessing = false; }

	void preprocessingStart() { _preprocessing = true; }

	virtual void performAfter() {}; // todo, not implemented.

	virtual void performBefore() {};

	virtual void perform() = 0;
};

class Action8x8Test : public ActionBase {
	void perform();
public:
	Action8x8Test(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "led", "Test 8x8", 1, ID_MRM_8x8A) {}
};

class ActionBluetoothTest : public ActionBase {
	void perform();
public:
	ActionBluetoothTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "blt", "Test Bluetooth", 16) {}
};

class ActionCANBusScan : public ActionBase {
	void perform();
public:
	ActionCANBusScan(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "can", "Report devices", 1) {}
};

class ActionCANBusSniff : public ActionBase {
	void perform();
public:
	ActionCANBusSniff(Robot* robot) : ActionBase(robot, "sni", "Sniff bus toggle", 16) {}
};

class ActionCANBusStress : public ActionBase {
	void perform();
public:
	ActionCANBusStress(Robot* robot) : ActionBase(robot, "all", "CAN Bus stress", 16) {}//1 | 2 | 4 | 8 | 16 | 32 | 64 | 128-> in all menus. 0 - in no menu.
};

class ActionColorBTest6Colors : public ActionBase {
	void perform();
public:
	ActionColorBTest6Colors(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "10c", "Test 10 colors", 4, ID_MRM_COL_B) {}
};

class ActionColorBTestHSV : public ActionBase {
	void perform();
public:
	ActionColorBTestHSV(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "hs1", "Test HSV", 4, ID_MRM_COL_B) {}
};

class ActionColorIlluminationOff : public ActionBase {
	void perform();
public:
	ActionColorIlluminationOff(Robot* robot) : ActionBase(robot, "lof", "Light off", 4, ID_MRM_COL_CAN) {}
};

class ActionColorIlluminationOn : public ActionBase {
	void perform();
public:
	ActionColorIlluminationOn(Robot* robot) : ActionBase(robot, "lon", "Light on", 4, ID_MRM_COL_CAN) {}
};

class ActionColorPatternErase : public ActionBase {
	void perform();
public:
	ActionColorPatternErase(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "per", "Erase patterns", 4, ID_MRM_COL_CAN) {}
};

class ActionColorPatternPrint : public ActionBase {
	void perform();
public:
	ActionColorPatternPrint(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "ppr", "Print patterns", 4, ID_MRM_COL_CAN) {}
};

class ActionColorPatternRecognize : public ActionBase {
	void perform();
public:
	ActionColorPatternRecognize(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "pre", "Recognize pattern", 4, ID_MRM_COL_CAN) {}
};

class ActionColorPatternRecord : public ActionBase {
	void perform();
public:
	ActionColorPatternRecord(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "par", "Record patterns", 4, ID_MRM_COL_CAN) {}
};

class ActionColorTest6Colors : public ActionBase {
	void perform();
public:
	ActionColorTest6Colors(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "6co", "Test 6 colors", 4, ID_MRM_COL_CAN) {}
};

class ActionColorTestHSV : public ActionBase {
	void perform();
public:
	ActionColorTestHSV(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "hsv", "Test HSV", 4, ID_MRM_COL_CAN) {}
};

class ActionDeviceIdChange : public ActionBase {
	void perform();
public:
	ActionDeviceIdChange(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "idc", "Id change", 1) {}
};

class ActionDoNothing : public ActionBase {
	void perform() {}
public:
	ActionDoNothing(Robot* robot) : ActionBase(robot, "", "No action", 0) {}
};

class ActionFirmware : public ActionBase {
	void perform();
public:
	ActionFirmware(Robot* robot) : ActionBase(robot, "fir", "Firmware", 16) {}
};

class ActionFPS : public ActionBase {
	void perform();
public:
	ActionFPS(Robot* robot) : ActionBase(robot, "fps", "FPS", 16) {}
};

class ActionGoAhead : public ActionBase {
	void perform();
public:
	ActionGoAhead(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "ahe", "Go ahead", 1) {}
};

class ActionI2CTest : public ActionBase {
	void perform();
public:
	ActionI2CTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "i2c", "Test I2C", 16) {}
};

class ActionIMUTest : public ActionBase {
	void perform();
public:
	ActionIMUTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "imu", "Test IMU", 1) {}
};

class ActionInfo : public ActionBase {
	void perform();
public:
	ActionInfo(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "inf", "Info", 1) {}
};

class ActionIRFinderTest : public ActionBase {
	void perform();
public:
	ActionIRFinderTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "irf", "Test ball analog", 1, ID_MRM_IR_FINDER_2) {}
};

class ActionIRFinderCanTest : public ActionBase {
	void perform();
public:
	ActionIRFinderCanTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "irs", "Test ball raw", 1, ID_MRM_IR_FINDER3) {}
};

class ActionIRFinderCanTestCalculated : public ActionBase {
	void perform();
public:
	ActionIRFinderCanTestCalculated(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "irc", "Test ball calcul.", 1, ID_MRM_IR_FINDER3) {}
};

class ActionLidar2mTest : public ActionBase {
	void perform();
public:
	ActionLidar2mTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "li2", "Test li. 2m", 1, ID_MRM_LID_CAN_B) {}
};

class ActionLidar4mTest : public ActionBase {
	void perform();
public:
	ActionLidar4mTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "li4", "Test li. 4m", 1, ID_MRM_LID_CAN_B2){}
};

class ActionLidar4mMultiTest : public ActionBase {
	void perform();
public:
	ActionLidar4mMultiTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "lim", "Test li. mul", 1, ID_MRM_LID_D){}
};

class ActionLidarCalibrate : public ActionBase {
	void perform();
public:
	ActionLidarCalibrate(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "lic", "Cal. lidar", 1) {}
};

class ActionLoop : public ActionBase {
	void perform();
public:
	ActionLoop(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "loo", "Loop test", 1, ID_ANY, ledSign) {}
};

class ActionMenuColor : public ActionBase {
	void perform();
public:
	ActionMenuColor(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "col", "Color (menu)", 1, ID_MRM_COL_CAN) {}
};

class ActionMenuColorB : public ActionBase {
	void perform();
public:
	ActionMenuColorB(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "col", "Color (menu)", 1, ID_MRM_COL_B) {}
};

class ActionMenuMain : public ActionBase {
	void perform();
public:
	ActionMenuMain(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "x", "Escape", 2 | 4 | 8 | 16 | 32 | 64 | 128) {}//2 | 4 | 8 | 16 -> in all menus except 1. 0 - in no menu.
};

class ActionMenuReflectance : public ActionBase {
	void perform();
public:
	ActionMenuReflectance(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "ref", "Reflect. (menu)", 1, ID_MRM_REF_CAN) {}
};

class ActionMenuSystem : public ActionBase {
	void perform();
public:
	ActionMenuSystem(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "sys", "System (menu)", 1) {}
};

class ActionMotorTest : public ActionBase {
	void perform();
public:
	ActionMotorTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "mot", "Test motors", 1) {}
};

class ActionNodeTest : public ActionBase {
	void perform();
public:
	ActionNodeTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "nod", "Test node", 1, ID_MRM_NODE) {}
};

class ActionNodeServoTest : public ActionBase {
	void perform();
public:
	ActionNodeServoTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "nos", "Test node servo", 1, ID_MRM_NODE) {}
};

//class ActionOscillatorTest : public ActionBase {
//	void perform();
//public:
//	ActionOscillatorTest(Robot* robot) : ActionBase(robot, "osc", "Oscillator test", 1, ID_ANY) {}
//};

class ActionPnPOff : public ActionBase {
	void perform();
public:
	ActionPnPOff(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "pof", "PnP off", 16) {}
};

class ActionPnPOn : public ActionBase {
	void perform();
public:
	ActionPnPOn(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "pon", "PnP on", 16) {}
};

class ActionStop : public ActionBase {
	void perform();
public:
	ActionStop(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "sto", "Stop", 1) {}
};

class ActionReflectanceArrayAnalogTest : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayAnalogTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "anr", "Test refl. anal.", 2, ID_MRM_REF_CAN) {}
};

class ActionReflectanceArrayDigitalTest : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayDigitalTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "dgr", "Test refl. digi.", 2, ID_MRM_REF_CAN) {}
};

class ActionReflectanceArrayCalibrate : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayCalibrate(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "cal", "Calibrate refl.", 2, ID_MRM_REF_CAN) {}
};

class ActionReflectanceArrayCalibrationPrint : public ActionBase {
	void perform();
public:
	ActionReflectanceArrayCalibrationPrint(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "pri", "Calibration print", 2, ID_MRM_REF_CAN) {}
};

class ActionServoTest : public ActionBase {
	void perform();
public:
	ActionServoTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "ser", "Test servo", 1) {}
};

class ActionServoInteractive : public ActionBase {
	void perform();
public:
	ActionServoInteractive(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "ses", "Set servo", 1) {}
};

class ActionThermoTest : public ActionBase {
	void perform();
public:
	ActionThermoTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "the", "Test thermo", 1, ID_MRM_THERM_B_CAN) {}
};

class ActionUS_BTest : public ActionBase {
	void perform();
public:
	ActionUS_BTest(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "uls", "Test ultras.", 1, ID_MRM_US_B) {}
};

class ActionUS1Test : public ActionBase {
	void perform();
public:
	ActionUS1Test(Robot* robot, LEDSign* ledSign = NULL) : ActionBase(robot, "ult", "Test ultras.", 1, ID_MRM_US1) {}
};
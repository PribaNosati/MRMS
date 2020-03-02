#include <mrm-action.h>
#include <mrm-8x8a.h>
#include <mrm-imu.h>
#include <mrm-node.h>
#include <mrm-ir-finder2.h>
#include <mrm-ir-finder-can.h>
#include <mrm-ref-can.h>
#include <mrm-servo.h>
#include <mrm-robot.h>

ActionBase::ActionBase(Robot* robot, char shortcut[4], char text[20], uint8_t menuLevel) {
	_robot = robot;
	if (shortcut != 0)
		strcpy(_shortcut, shortcut);
	if (text != 0)
		strcpy(_text, text);
	_menuLevel = menuLevel;
}

void Action8x8Test::perform() { _robot->mrm_8x8a->test(); }
void ActionAny::perform() { _robot->anyTest(); }
void ActionBluetoothTest::perform() { _robot->bluetoothTest(); }
void ActionCANBusScan::perform() { _robot->devicesScan(true); }
void ActionCANBusSniff::perform() { _robot->canBusSniff(); }
void ActionCANBusStress::perform() { _robot->stressTest(); }
void ActionColorTest::perform() { _robot->colorTest(); }
void ActionDeviceIdChange::perform() { _robot->canIdChange(); }
void ActionFirmware::perform() { _robot->firmwarePrint(); }
void ActionFPS::perform() { _robot->fpsPrint(); }
void ActionGoAhead::perform() { _robot->goAhead(); }
void ActionI2CTest::perform() { _robot->i2cTest(); }
void ActionIRFinderTest::perform() { _robot->mrm_ir_finder2->test(); }
void ActionIRFinderCanTest::perform() { _robot->irFinderCanTest(); }
void ActionIRFinderCanTestCalculated::perform() { _robot->irFinderCanTestCalculated(); }
void ActionIMUTest::perform() { _robot->mrm_imu->test(); }
void ActionLidarCalibrate::perform() { _robot->lidarCalibrate(); }
void ActionLidar2mTest::perform() { _robot->lidar2mTest(); }
void ActionLidar4mTest::perform() { _robot->lidar4mTest(); }
void ActionMenuMain::perform() { _robot->menuMainAndIdle(); }
void ActionMotorTest::perform() { _robot->motorTest(); }
void ActionNodeTest::perform() { _robot->nodeTest(); }
void ActionNodeServoTest::perform() { _robot->mrm_node->servoTest();}
void ActionReflectanceArrayTest::perform() { _robot->reflectanceArrayTest(); }
void ActionReflectanceArrayCalibrate::perform() { _robot->mrm_ref_can->calibrate(); }
void ActionReflectanceArrayCalibrationPrint::perform() { _robot->reflectanceArrayCalibrationPrint(); }
void ActionServoTest::perform() { _robot->mrm_servo->test(); }
void ActionStop::perform() { _robot->stopAll(); }
void ActionThermoTest::perform() { _robot->thermoTest(); }

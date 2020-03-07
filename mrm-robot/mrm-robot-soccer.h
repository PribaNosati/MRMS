#pragma once
#include <mrm-action.h>
#include <mrm-pid.h>
#include <mrm-robot.h>

class RobotSoccer : public Robot {
	ActionBase* actionCatch;
	ActionBase* actionIdle;
	float headingToMaintain;
	MotorGroupStar* motorGroup;
	Mrm_pid* pidXY; // PID controller, regulates motors' speeds for linear motion in the x-y plane
	Mrm_pid* pidRotation; // PID controller, regulates rotation around z axis
public:
	RobotSoccer();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	void catchBall();
	void goAhead();
	void idle();
	void play();

	void anyTest();
};

class ActionSoccerCatch : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->catchBall(); }
public:
	ActionSoccerCatch(RobotSoccer* robot) : ActionBase(robot, "cat", "Soccer catch", 0) {
		_robot = robot;
	}
};

class ActionSoccerPlay : public ActionBase {
	void perform(){ ((RobotSoccer*)_robot)->play(); }
public:
	ActionSoccerPlay(RobotSoccer* robot) : ActionBase(robot, "soc", "Soccer play") {
		_robot = robot;
	}
};

class ActionSoccerIdle : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->play(); }
public:
	ActionSoccerIdle(RobotSoccer* robot) : ActionBase(robot, "idl", "Soccer idle", 0) {
		_robot = robot;
	}
};

#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

class RobotLine : public Robot {
	MotorGroupDifferential* motorGroup = NULL;
public:
	RobotLine();

	void goAhead();

	void lineFollow();

	void omniWheelsTest();
};

class ActionLineFollow : public ActionBase {
	void perform(){ ((RobotLine*)_robot)->lineFollow(); }
public:
	ActionLineFollow(RobotLine* robot) : ActionBase(robot, "lin", "Line follow") {
		_robot = robot;
	}
};

class ActionOmniWheelsTest : public ActionBase {
	void perform() { ((RobotLine*)_robot)->omniWheelsTest(); }
public:
	ActionOmniWheelsTest(Robot* robot) : ActionBase(robot, "omn", "Test omni wheels", 1) {
		_robot = robot;
	}
};
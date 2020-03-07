#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

class RobotLine : public Robot {
	MotorGroupDifferential* motorGroup = NULL;
	ActionBase* actionObstacleAvoid;
	ActionBase* actionLineFollow;
	ActionBase* actionWallFollow;
public:
	RobotLine();

	void anyTest();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	void display(uint8_t bitmap);

	void goAhead();

	void lineFollow();

	void obstacleAvoid();

	void omniWheelsTest();

	void wallFollow();
};

class ActionLineFollow : public ActionBase {
	void perform(){ ((RobotLine*)_robot)->lineFollow(); }
public:
	ActionLineFollow(RobotLine* robot) : ActionBase(robot, "lin", "Line follow") {
		_robot = robot;
	}
};

class ActionObstacleAvoid : public ActionBase {
	void perform() { ((RobotLine*)_robot)->obstacleAvoid(); }
public:
	ActionObstacleAvoid(RobotLine* robot) : ActionBase(robot, "obs", "Obstacle avoid", 0) {
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

class ActionWallFollow : public ActionBase {
	void perform() { ((RobotLine*)_robot)->wallFollow(); }
public:
	ActionWallFollow(RobotLine* robot) : ActionBase(robot, "wall", "Wall follow") {
		_robot = robot;
	}
};
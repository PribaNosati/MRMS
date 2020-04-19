#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

// For mrm-8x8 display.
#define LED_CROSSING_LEFT_RIGHT 0
#define LED_LINE_FULL 1
#define LED_LINE_INTERRUPTED 2
#define LED_CURVE_LEFT 3
#define LED_CURVE_RIGHT 4
#define LED_OBSTACLE 5
#define LED_OBSTACLE_AROUND 6
#define LED_PAUSE 7
#define LED_PLAY 8

/** Robot for RCJ Rescue Line and Maze
*/
class RobotLine : public Robot {
	MotorGroupDifferential* motorGroup = NULL;
	ActionBase* actionObstacleAvoid;
	ActionBase* actionLineFollow;
	ActionBase* actionWallFollow;
public:
	RobotLine();

	/** Custom test
	*/
	void anyTest();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Displays a bitmap using mrm-8x8.
	@param bitmap - user bitmap's id
	*/
	void display(uint8_t bitmap);

	/** Test - go straight ahead using a defined speed.
	*/
	void goAhead();

	/** Follow a RCJ line.
	*/
	void lineFollow();

	/** Avoid an obstacle on line
	*/
	void obstacleAvoid();

	/** Test for Mecanum wheels.
	*/
	void omniWheelsTest();

	/** Starts the robot after this action selected.
	*/
	void rcjLine();

	void wallFollow();
};

// Actions specific for a RobotLine robot.

class ActionLineFollow : public ActionBase {
	void perform(){ ((RobotLine*)_robot)->lineFollow(); }
public:
	ActionLineFollow(RobotLine* robot) : ActionBase(robot, "lnf", "Line follow", 0) {}
};

class ActionObstacleAvoid : public ActionBase {
	void perform() { ((RobotLine*)_robot)->obstacleAvoid(); }
public:
	ActionObstacleAvoid(RobotLine* robot) : ActionBase(robot, "obs", "Obstacle avoid", 0) {}
};

class ActionOmniWheelsTest : public ActionBase {
	void perform() { ((RobotLine*)_robot)->omniWheelsTest(); }
public:
	ActionOmniWheelsTest(Robot* robot) : ActionBase(robot, "omn", "Test omni wheels", 1) {}
};

class ActionRCJLine : public ActionBase {
	void perform() { ((RobotLine*)_robot)->rcjLine(); }
public:
	ActionRCJLine(Robot* robot) : ActionBase(robot, "lin", "RCJ Line", 1) {}
};

class ActionWallFollow : public ActionBase {
	void perform() { ((RobotLine*)_robot)->wallFollow(); }
public:
	ActionWallFollow(RobotLine* robot) : ActionBase(robot, "wal", "Wall follow") {}
};
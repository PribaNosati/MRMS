#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

// For mrm-8x8 display.
enum ledSign { LED_CROSSING_BOTH_MARKS, LED_CROSSING_MARK_LEFT, LED_CROSSING_MARK_RIGHT, LED_CROSSING_NO_MARK, LED_LINE_FULL, LED_LINE_FULL_BOTH_MARKS, LED_LINE_FULL_MARK_LEFT, LED_LINE_FULL_MARK_RIGHT, 
	LED_LINE_INTERRUPTED, LED_CURVE_LEFT, LED_CURVE_RIGHT, LED_OBSTACLE, LED_OBSTACLE_AROUND_LEFT, LED_OBSTACLE_AROUND_RIGHT, LED_PAUSE, LED_PLAY };

class ActionObstacleAvoid;
class ActionLineFollow;

/** Robot for RCJ Rescue Line and Maze
*/
class RobotLine : public Robot {
	const uint8_t TOP_SPEED = 50; //90

	MotorGroupDifferential* motorGroup = NULL;
	ActionObstacleAvoid* actionObstacleAvoid;
	ActionLineFollow* actionLineFollow;
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

	/** Turns the robot clockwise using compass
	*/
	void turn(int16_t byDegreesClockwise);

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
	bool leftOfObstacle;

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
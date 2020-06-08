#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

// For mrm-8x8 display.
enum Direction{UP, LEFT, DOWN, RIGHT, NOWHERE};
enum LedSign { MAZE_LED_PAUSE, MAZE_LED_PLAY };
enum WallStatus{ NO_WALL, WALL_UNKNOWN, WALL_WITH_VICTIM, WALL_WITHOUT_VICTIM};
enum TileSurvace{ SURFACE_BLACK, SURFACE_SILVER, SURFACE_UNKNOWN, SURFACE_WHITE};

class Tile {

	uint8_t _wall; // Bit 7:6 - right, 5:4 - down, 3:2 - left, 1:0 - up
	uint8_t _surface; // Bit 7:6 - surface
	Tile* _chain;

public:
	static Tile* last;

	Direction breadcrumb;
	int8_t x;
	int8_t y;

	Tile(int8_t x, int8_t y, Direction breadcrumb);

	Tile* next() { return _chain; }

	WallStatus wallGet(Direction direction) { return (WallStatus)((_wall >> (direction * 2)) & 0b11); }

	void wallSet(Direction direction, WallStatus wallStatus) { _wall &= ~(0b11 << (direction * 2)) & (wallStatus << (direction * 2)); }
};

/** Robot for RCJ Rescue Maze
*/
class RobotMaze : public Robot {
	const uint16_t IMU_FOLLOW_STRENGTH = 5;
	const uint16_t NO_WALL_DISTANCE = 250; // mm
	const uint8_t TOP_SPEED = 50;
	const uint16_t WALL_FOLLOW_ERROR_ALLOWED = 40; // mm
	const uint16_t WALL_FOLLOW_STRENGTH = 2;

	ActionBase* actionDecide;
	ActionBase* actionGoStraightAhead;
	ActionBase* actionMap;
	ActionBase* actionMove;
	ActionBase* actionMoveAhead;
	ActionBase* actionMoveTurn;

	Direction directionCurrent;
	Direction directionToGo;

	float imuLastValid = 9999;

	MotorGroupDifferential* motorGroup = NULL;

	Tile* tileStart;
	Tile* tileCurrent;
public:
	RobotMaze();

	/** Custom test
	*/
	void anyTest();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	void decide();

	/** Displays a bitmap using mrm-8x8.
	@param bitmap - user bitmap's id
	*/
	void display(uint8_t bitmap);

	/** Distance, sensors: 0 FR, 1 FL, 2 LF, 3 LB, 4 BR, 5 BL, 6 RB, 7 RF
	*/
	uint16_t distance(Direction direction, bool firstAntiCW) { return mrm_lid_can_b->reading(2*((direction-directionCurrent) % 4) + firstAntiCW); } // -3 % 4 mora biti 1!

	/** Orders the robot to go ahead
	*/
	void goAhead() {}// todo

	void imuFollow();

	void map();

	void move();

	void moveAhead();

	void moveTurn();

	/** Test for Mecanum wheels.
	*/
	void omniWheelsTest();

	void rescueMaze();

	Tile* tileContaining(int8_t x, int8_t y);

	/** Turns the robot clockwise using compass
	*/
	void turn(int16_t byDegreesClockwise);

	Direction wallClosest();

	void wallFollow(Direction wallDirection);
};

// Actions specific for a RobotMaze robot.

class ActionDecide : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->decide(); }
public:
	ActionDecide(Robot* robot) : ActionBase(robot, "", "") {}
};

class ActionMap : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->map(); }
public:
	ActionMap(Robot* robot) : ActionBase(robot, "", "") {}
};

class ActionMove : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->move(); }
public:
	ActionMove(Robot* robot) : ActionBase(robot, "", "") {}
};

class ActionMoveAhead : public ActionMove {
	void perform() { ((RobotMaze*)_robot)->moveAhead(); }
public:
	ActionMoveAhead(Robot* robot) : ActionMove(robot) {}
};

class ActionMoveTurn : public ActionMove {
	void perform() { ((RobotMaze*)_robot)->moveTurn(); }
public:
	ActionMoveTurn(Robot* robot) : ActionMove(robot) {}
};

class ActionRescueMaze : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->rescueMaze(); }
public:
	ActionRescueMaze(Robot* robot) : ActionBase(robot, "maz", "Rescue Maze", 1) {}
};

class ActionOmniWheelsTest : public ActionBase {
	void perform() { ((RobotMaze*)_robot)->omniWheelsTest(); }
public:
	ActionOmniWheelsTest(Robot* robot) : ActionBase(robot, "omn", "Test omni wheels", 1) {}
};

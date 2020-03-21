#pragma once
#include <mrm-action.h>
#include <mrm-pid.h>
#include <mrm-robot.h>

/** Robot for RCJ Soccer
*/
class RobotSoccer : public Robot {
	ActionBase* actionCatch;
	ActionBase* actionIdle;
	float headingToMaintain; // Heading towards opponent's goal
	MotorGroupStar* motorGroup;
	Mrm_pid* pidXY; // PID controller, regulates motors' speeds for linear motion in the x-y plane
	Mrm_pid* pidRotation; // PID controller, regulates rotation around z axis
public:
	RobotSoccer();

	/** Custom test
	*/
	void anyTest();

	/** Store bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Go around the ball and approach it.
	*/
	void catchBall();

	/** Test - go straight ahead.
	*/
	void goAhead();

	/** No ball detected - return to Your goal.
	*/
	void idle();

	/** Starts robot
	*/
	void play();
};

// Custom actions

class ActionSoccerCatch : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->catchBall(); }
public:
	ActionSoccerCatch(RobotSoccer* robot) : ActionBase(robot, "cat", "Soccer catch", 0) {}
};

class ActionSoccerPlay : public ActionBase {
	void perform(){ ((RobotSoccer*)_robot)->play(); }
public:
	ActionSoccerPlay(RobotSoccer* robot) : ActionBase(robot, "soc", "Soccer play") {}
};

class ActionSoccerIdle : public ActionBase {
	void perform() { ((RobotSoccer*)_robot)->play(); }
public:
	ActionSoccerIdle(RobotSoccer* robot) : ActionBase(robot, "idl", "Soccer idle", 0) {}
};

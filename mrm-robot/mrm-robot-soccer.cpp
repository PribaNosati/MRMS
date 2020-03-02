#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
#include <mrm-ir-finder2.h>
#include <mrm-mot2x50.h>
#include "mrm-robot-Soccer.h"

RobotSoccer::RobotSoccer() : Robot() {
	motorGroup = new MotorGroupStar(mrm_mot2x50, 0, mrm_mot2x50, 1, mrm_mot2x50, 2, mrm_mot2x50, 3);

	pidXY = new Mrm_pid(0.5, 200, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane
	pidRotation = new Mrm_pid(0.5, 100, 0); // PID controller, regulates rotation around z axis
	actionAdd(new ActionSoccerPlay(this));
	actionCatch = new ActionSoccerCatch(this);
	actionIdle = new ActionSoccerIdle(this);
}

void RobotSoccer::catchBall() {
	if (mrm_ir_finder2->anyIRSource()) {

	}
	else
		actionSet(actionIdle);
}

void RobotSoccer::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed);
	_actionCurrent = NULL;
}

void RobotSoccer::idle() {
	if (_actionCurrent->_firstProcess)
		headingToMaintain = mrm_imu->heading();
	if (mrm_ir_finder2->anyIRSource() && false)
		actionSet(actionCatch);
	else {
		//print("\n\rError: %i = %i - 300\n\r", (int)(300 - robot->mrm_lid_can_b2->reading(3)), robot->mrm_lid_can_b2->reading(3));
		float errorL = 900 - mrm_lid_can_b2->reading(3);
		float errorR = mrm_lid_can_b2->reading(1) - 900;
		motorGroup->goToEliminateErrors(errorL > errorR ? errorL : errorR, 150 - mrm_lid_can_b2->reading(2), headingToMaintain - mrm_imu->heading(), pidXY, pidRotation, true);
		//delay(500);
	}
}

void RobotSoccer::play() {
	if (motorGroup == NULL) {
		print("Define robot->motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu->heading();
	broadcastingStart();
	actionSet(actionIdle);
}
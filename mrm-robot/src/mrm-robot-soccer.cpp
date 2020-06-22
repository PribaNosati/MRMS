#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
#include <mrm-ir-finder2.h>
#include <mrm-mot2x50.h>
#include "mrm-robot-soccer.h"

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotSoccer::RobotSoccer(char name[]) : Robot(name) {
	motorGroup = new MotorGroupStar(mrm_mot2x50, 0, mrm_mot2x50, 1, mrm_mot2x50, 2, mrm_mot2x50, 3);

	pidXY = new Mrm_pid(0.5, 200, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane
	pidRotation = new Mrm_pid(0.5, 100, 0); // PID controller, regulates rotation around z axis
	actionAdd(new ActionSoccerPlay(this));
	actionCatch = new ActionSoccerCatch(this);
	actionIdle = new ActionSoccerIdle(this);
}

/** Custom test.
*/
void RobotSoccer::anyTest() {
	if (actionPreprocessing(true)) {
		mrm_ref_can->start(0xFF, 1, 15);
		mrm_bldc2x50->start(0xFF, 0, 15);
		fpsPause();
	}
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
	uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
	uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
	mrm_8x8a->bitmapCustomStore(red, green, 7);
}

/** Go around the ball and approach it.
*/
void RobotSoccer::catchBall() {
	if (mrm_ir_finder2->anyIRSource()) {

	}
	else
		actionSet(actionIdle);
}

/** Test - go straight ahead.
*/
void RobotSoccer::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed);
	actionEnd();
}

/** No ball detected - return to Your goal.
*/
void RobotSoccer::idle() {
	if (actionPreprocessing(true))
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

/** Starts robot
*/
void RobotSoccer::play() {
	if (motorGroup == NULL) {
		print("Define robot->motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu->heading();
	devicesStart();
	actionSet(actionIdle);
}
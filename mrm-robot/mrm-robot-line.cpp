#include "mrm-robot-line.h"
#include <mrm-mot4x3.6can.h>

RobotLine::RobotLine() : Robot() {
	motorGroup = new MotorGroupDifferential(mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 3);
	actionAdd(new ActionLineFollow(this));
	actionAdd(new ActionOmniWheelsTest(this));
}

void RobotLine::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed, speed);
	_actionCurrent = NULL;
}

void RobotLine::lineFollow() {
	static float lastLineCenter = 0;
	//static uint32_t ms = 0;
	if (_actionCurrent->_firstProcess) // Only in the first pass.
		broadcastingStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.

	float lineCenter = (mrm_ref_can->center() - 4500) / 70.0; // Result: -50 to 50.
	//if (millis() - ms > 100) {
	//	print("C:%i R:%i %i %i %i %i %i %i %i %i %ims\n\r", (int)lineCenter, (int)mrm_ref_can->center(), (int)mrm_ref_can->dark(7),
	//		mrm_ref_can->dark(6), mrm_ref_can->dark(5), mrm_ref_can->dark(4), mrm_ref_can->dark(3), mrm_ref_can->dark(2), mrm_ref_can->dark(1),
	//		mrm_ref_can->dark(0), millis() - mrm_ref_can->lastMessageMs());
	//	ms = millis();
	//}
	if (lineCenter < -40 || lineCenter > 40) {// No line under inner sensors, including lost line.
		//robot->motorGroupDifferential->stop();
		//delay(100);
		// Choice depending on the line center the last time it was detected, stored in variable lastLineCenter.
		if (lastLineCenter < -15) // Lost line right or line far right.
			motorGroup->go(127, -127); // Rotate in place.
		else if (lastLineCenter > 15) // Lost line left or line far left.
			motorGroup->go(-127, 127); // Rotate in place.
		else // Line was around the robot's center when lost. Therefore, it was interrupted
			motorGroup->go(127, 127); // Go straight ahead.
	}
	else { // Follow line
		// Maximum speed of the faster motor, decrease the other one.
		motorGroup->go(lineCenter < 0 ? 127 : 127 - lineCenter * 3, lineCenter < 0 ? 127 + lineCenter * 3 : 127);
		lastLineCenter = lineCenter; // Remember the line.
	}
}

void RobotLine::omniWheelsTest() {
	static uint8_t nextMove;
	static uint32_t lastMs;
	if (_actionCurrent->_firstProcess) {
		if (motorGroup == NULL) {
			print("Differential motor group needed.");
			_actionCurrent = NULL;
			return;
		}
		nextMove = 0;
		lastMs = millis();
	}
	switch (nextMove) {
	case 0:
		if (millis() - lastMs > 2000) {
			lastMs = millis();
			nextMove = 1;
		}
		else
			motorGroup->go(20, 20, 70);
		break;
	case 1:
		if (millis() - lastMs > 15000) {
			lastMs = millis();
			nextMove = 2;
		}
		else {
			float angle = (millis() - lastMs) / 1500.0;
			int8_t x = cos(angle) * 50;
			int8_t y = sin(angle) * 50;
			motorGroup->go(y, y, x);
			//print("%i %i %i\n\r", (int)(angle*100), (int)(x * 100), (int)(y * 100));
			//delay(100);
		}
		break;
	case 2:
		if (millis() - lastMs > 2000) {
			lastMs = millis();
			nextMove = 0;
		}
		else
			motorGroup->go(-20, -20, -70);
		break;
	}
}

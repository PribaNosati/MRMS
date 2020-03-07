#include <mrm-8x8a.h>
#include <mrm-lid-can-b.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-line.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>

RobotLine::RobotLine() : Robot() {
	motorGroup = new MotorGroupDifferential(mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 3);
	
	actionLineFollow = new ActionLineFollow(this);
	actionObstacleAvoid = new ActionObstacleAvoid(this);
	actionWallFollow = new ActionWallFollow(this);

	actionAdd(actionLineFollow);
	actionAdd(actionObstacleAvoid);
	actionAdd(actionWallFollow);
	actionAdd(new ActionOmniWheelsTest(this));
	bitmapsSet();
}

void RobotLine::anyTest() {
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotLine::bitmapsSet() {
#define LED_CROSSING_LEFT_RIGHT 0
#define LED_LINE_FULL 1
#define LED_LINE_INTERRUPTED 2
#define LED_CURVE_LEFT 3
#define LED_CURVE_RIGHT 4
#define LED_OBSTACLE 5
#define LED_OBSTACLE_AROUND 6
#define LED_PAUSE 7
#define LED_PLAY 8

	mrm_8x8a->alive(0, true);
	uint8_t red[8];
	uint8_t green[8];
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0, green[i] = 0;

	// Full line
	for (uint8_t i = 0; i < 8; i++)
		green[i] = 0b00011000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL);
	delayMs(1);

	// Crossing left-right
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11111111;
	green[3] = 0b11111111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CROSSING_LEFT_RIGHT);
	delayMs(1);

	// Interrupted line
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b00000000;
	green[3] = 0b00000000;
	green[4] = 0b00000000;
	green[5] = 0b00000000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_INTERRUPTED);
	delayMs(1);

	// Curve left
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11111000;
	green[3] = 0b11111000;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CURVE_LEFT);
	delayMs(1);

	// Curve left
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b00011111;
	green[3] = 0b00011111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CURVE_RIGHT);
	delayMs(1);

	// Obstacle
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b00000000;
	green[3] = 0b00011000;
	green[4] = 0b00111100;
	green[5] = 0b01111110;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_OBSTACLE);
	delayMs(1);

	// Around obstacle
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b00000011;
	green[3] = 0b00100011;
	green[4] = 0b01110000;
	green[5] = 0b11111000;
	green[6] = 0b00110000;
	green[7] = 0b00110000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_OBSTACLE_AROUND);
	delayMs(1);

	// Pause
	green[0] = 0b11100111;
	green[1] = 0b11100111;
	green[2] = 0b11100111;
	green[3] = 0b11100111;
	green[4] = 0b11100111;
	green[5] = 0b11100111;
	green[6] = 0b11100111;
	green[7] = 0b11100111;
	mrm_8x8a->bitmapCustomStore(red, green, LED_PAUSE);
	delayMs(1);

	// Play
	green[0] = 0b0110000;
	green[1] = 0b0111000;
	green[2] = 0b0111100;
	green[3] = 0b0111110;
	green[4] = 0b0111110;
	green[5] = 0b0111100;
	green[6] = 0b0111000;
	green[7] = 0b0110000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_PLAY);
	delayMs(10);

}

void RobotLine::display(uint8_t bitmap) {
	uint8_t lastBitmap = 0xFF;
	uint32_t lastMs = 0;
	if (bitmap != lastBitmap && millis() - lastMs > 100) {
		mrm_8x8a->bitmapCustomStoredDisplay(bitmap);
		lastMs = millis();
		lastBitmap = bitmap;
	}
}

void RobotLine::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed, speed);
	actionEnd();
}

void RobotLine::lineFollow() {
	delayMs(2); // Reduce number od CAN Bus messages sent.
	
	const uint16_t MS_IN_THE_PAST = 100;//250
	const uint8_t TOP_SPEED = 90; //70 127
	const uint8_t TURNING_STRENGTH = 4; //3 3

	static uint32_t interruptStartedMs = 0;
	static uint8_t lastPrinted = 0;
	static uint32_t lastCurveLMs = 0;
	static uint32_t lastCurveRMs = 0;
	static uint32_t ms = 0;
	if (actionInitialization(true)) { // Only in the first pass.
		display(LED_PLAY);
		devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.
		delayMs(200);
	}

	//if (millis() - ms > 100) {
	//	print("C:%i %i %i %i %i %i %i %i %i M%i L%i R%ims\n\r", (int)(mrm_ref_can->center() - 4500), mrm_ref_can->dark(7),
	//		mrm_ref_can->dark(6), mrm_ref_can->dark(5), mrm_ref_can->dark(4), mrm_ref_can->dark(3), mrm_ref_can->dark(2), mrm_ref_can->dark(1),
	//		mrm_ref_can->dark(0), millis() - mrm_ref_can->lastMessageMs(), millis() - lastCurveLMs, millis() - lastCurveRMs);
	//	ms = millis();
	//}

	// Obstacle?
	if (mrm_lid_can_b->reading(0) < 50 && mrm_lid_can_b->reading(0) != 0) {
		print("Obstacle: %i\n\r", mrm_lid_can_b->reading(0));
		actionSet(actionObstacleAvoid);
		return;
	}

	// Sharp curve left and right?
	bool any = false; // Any transistor senses dark (line)
	for (int8_t i = 0; i < 8; i++)
		if (mrm_ref_can->dark(i)) {
			if (i == 0 || i == 1)
				lastCurveRMs = millis();
			if (i == 6 || i == 7)
				lastCurveLMs = millis();
			any = true;
		}

	if (any)  { // Follow line. Maximum speed of the faster motor, decrease the other one.
		float lineCenter = (mrm_ref_can->center() - 4500) / 70.0; // Result: -50 to 50.
		motorGroup->go(lineCenter < 0 ? TOP_SPEED : TOP_SPEED - lineCenter * 3, lineCenter < 0 ? TOP_SPEED + lineCenter * 3 : TOP_SPEED);
		//lastPrinted = 0;
		interruptStartedMs = 0;
		display(LED_LINE_FULL);
	}
	else {// No line 
		if (interruptStartedMs == 0)
			interruptStartedMs = millis();
		//robot->motorGroup->stop();
		//delay(100);
		// Choice depending on the line center the last time it was detected, stored in variable lastLineCenter.
		if (millis() - lastCurveLMs < MS_IN_THE_PAST && millis() - lastCurveRMs < MS_IN_THE_PAST) {
			//print("STOP %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
			motorGroup->stop();
			actionEnd();
			display(LED_CROSSING_LEFT_RIGHT);
		}
		else if (millis() - lastCurveLMs < MS_IN_THE_PAST) { // Last sharp curve was left
			//if (lastPrinted != 1)
			//	print("Left %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
			//lastPrinted = 1;
			motorGroup->go(-TOP_SPEED, TOP_SPEED);	// Rotate in place.
			lastCurveLMs = millis();
			display(LED_CURVE_LEFT);
		}
		else if (millis() - lastCurveRMs < MS_IN_THE_PAST) { // Last sharp curve was right
			//if (lastPrinted != 2)
			//	print("Right %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
			//lastPrinted = 2;
			motorGroup->go(TOP_SPEED, -TOP_SPEED);	// Rotate in place.
			lastCurveRMs = millis();
			display(LED_CURVE_RIGHT);
		}
		else {							// No sharp curve in the near past. Therefore, straight line was interrupted.
			//if (lastPrinted != 3)
			//	print("Ahead %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
			//lastPrinted = 3;
			motorGroup->go(TOP_SPEED, TOP_SPEED);	// Go straight ahead.
			display(LED_LINE_INTERRUPTED);
			if (millis() - interruptStartedMs > 2000) {
				display(LED_PAUSE);
				actionEnd();
				motorGroup->stop();
			}
		}
	}
}

void RobotLine::obstacleAvoid() {
	static uint8_t part = 0;
	static uint32_t startMs = 0;
	if (actionInitialization(true))
		display(LED_OBSTACLE);

	switch (part) {
	case 0: // Turn in place
		if (mrm_lid_can_b->reading(0) < 60 || mrm_ref_can->any(true))
			motorGroup->go(-100, 100);
		else {
			part = 1;
			startMs = millis();
		}
		break;
	case 1: // Continue turning even more
		if (millis() - startMs > 200) {
			part = 2;
			display(LED_OBSTACLE_AROUND);
		}
		break;
	case 2: // Go around obstacle
		if (mrm_ref_can->any())
			part = 3;
		else
			motorGroup->go(90, 30);
		break;
	case 3: // Follow line again
	default:
		part = 0;
		actionSet(actionLineFollow);
		actionInitializationFinish();
		break;
	}
}

void RobotLine::omniWheelsTest() {
	static uint8_t nextMove;
	static uint32_t lastMs;
	if (actionInitialization(true)) {
		if (motorGroup == NULL) {
			print("Differential motor group needed.");
			actionEnd();
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

void RobotLine::wallFollow() {
	static uint32_t ms = 0;
	if (actionInitialization(true)) { // Only in the first pass.
		display(LED_PLAY);
		devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.
	}

	if (millis() - ms > 100)
		print("%i\n\r", mrm_lid_can_b->reading());

	if (mrm_lid_can_b->reading() < 100) {
		motorGroup->go(20, 80);
	}
	else {
		motorGroup->go(80, 20);
	}
}
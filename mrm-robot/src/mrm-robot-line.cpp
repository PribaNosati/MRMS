#include <mrm-8x8a.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
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
	actionAdd(new ActionRCJLine(this));
}

/** Custom test
*/
void RobotLine::anyTest() {
	static bool yes = true;
	if (actionPreprocessing(true)) {
		mrm_lid_can_b->reset();
		delay(1);
	}
	actionEnd();
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotLine::bitmapsSet() {
	mrm_8x8a->alive(0, true);
	uint8_t red[8];
	uint8_t green[8];
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0, green[i] = 0;

	// Full line, no marks
	for (uint8_t i = 0; i < 8; i++)
		green[i] = 0b00011000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL);
	delayMs(1);

	// Full line, both marks
	for (uint8_t i = 0; i < 8; i++)
		green[i] = 0b00011000;
	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b01100110;
	red[5] = 0b01100110;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL_BOTH_MARKS);
	delayMs(1);

	// Full line, left mark
	for (uint8_t i = 0; i < 8; i++)
		green[i] = 0b00011000;
	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b01100000;
	red[5] = 0b01100000;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL_MARK_LEFT);
	delayMs(1);

	// Full line, right mark
	for (uint8_t i = 0; i < 8; i++)
		green[i] = 0b00011000;
	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b00000110;
	red[5] = 0b00000110;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL_MARK_RIGHT);
	delayMs(1);

	// Crossing both marks
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11111111;
	green[3] = 0b11111111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;

	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b01100110;
	red[5] = 0b01100110;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CROSSING_BOTH_MARKS);
	delayMs(1);

	// Crossing mark left
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11111111;
	green[3] = 0b11111111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;

	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b01100000;
	red[5] = 0b01100000;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CROSSING_MARK_LEFT);
	delayMs(1);

	// Crossing mark right
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11111111;
	green[3] = 0b11111111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;

	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b00000110;
	red[5] = 0b00000110;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CROSSING_MARK_RIGHT);
	delayMs(1);

	// Crossing, no marks
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11111111;
	green[3] = 0b11111111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_CROSSING_NO_MARK);
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_OBSTACLE);
	delayMs(1);

	// Around obstacle left
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b00000011;
	green[3] = 0b00100011;
	green[4] = 0b01110000;
	green[5] = 0b11111000;
	green[6] = 0b01110000;
	green[7] = 0b01110000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_OBSTACLE_AROUND_LEFT);
	delayMs(1);

	// Around obstacle right
	green[0] = 0b00000000;
	green[1] = 0b00000000;
	green[2] = 0b11000000;
	green[3] = 0b11000100;
	green[4] = 0b00001110;
	green[5] = 0b00011111;
	green[6] = 0b00001110;
	green[7] = 0b00001110;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_OBSTACLE_AROUND_RIGHT);
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_PLAY);
	delayMs(10);

}

/** Displays a bitmap using mrm-8x8.
@param bitmap - user bitmap's id
*/
void RobotLine::display(uint8_t bitmap) {
	static uint8_t lastDisplayeBitmap = 0xFF;
	static uint32_t lastMs = 0;
	if (bitmap != lastDisplayeBitmap && millis() - lastMs > 10) {
		mrm_8x8a->bitmapCustomStoredDisplay(bitmap);
		lastMs = millis();
		lastDisplayeBitmap = bitmap;
	}
}

/** Test - go straight ahead using a defined speed.
*/
void RobotLine::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed, speed);
	actionEnd();
}

/** Follow a RCJ line.
*/
void RobotLine::lineFollow() {

	const uint16_t CURVE_BEFORE_MS = 200;//100 for speed 90
	const uint16_t CROSSING_DURATION_MS = 100;
	const uint16_t GREEN_BEFORE_MS = 400;
	const uint8_t TURNING_STRENGTH = 4; //3 3

	static uint32_t interruptStartedMs = 0;
	static uint8_t lastPrinted = 0;
	static uint32_t lastCurveLMs = 0;
	static uint32_t lastCurveRMs = 0;
	static uint32_t lastGreenLeftMs = 0;
	static uint32_t lastGreenRightMs = 0;
	static uint32_t enteredCrossingAtMs = 0;
	static uint32_t ms = 0;

	//if (millis() - ms > 100) {
	//	print("C:%i %i %i %i %i %i %i %i %i M%i L%i R%ims\n\r", (int)(mrm_ref_can->center() - 4500), mrm_ref_can->dark(7),
	//		mrm_ref_can->dark(6), mrm_ref_can->dark(5), mrm_ref_can->dark(4), mrm_ref_can->dark(3), mrm_ref_can->dark(2), mrm_ref_can->dark(1),
	//		mrm_ref_can->dark(0), millis() - mrm_ref_can->lastMessageMs(), millis() - lastCurveLMs, millis() - lastCurveRMs);
	//	ms = millis();
	//}

	// Obstacle?
	if (mrm_lid_can_b->reading(1) < 50 && mrm_lid_can_b->reading(1) != 0) { // Front sensor (1)
		print("Obstacle: %i\n\r", mrm_lid_can_b->reading(1));
		motorGroup->stop();
		delayMs(50);
		if (mrm_lid_can_b->reading(1) < 50) { // Check one more time
			actionSet(actionObstacleAvoid);
			actionObstacleAvoid->leftOfObstacle = (mrm_lid_can_b->reading(0) > mrm_lid_can_b->reading(2));
			return;
		}
	}

	// Green?
	if (mrm_col_can->patternRecognizedBy6Colors(0) == 2) // Learned pattern 2 is green
		lastGreenLeftMs = millis();
	if (mrm_col_can->patternRecognizedBy6Colors(1) == 2)
		lastGreenRightMs = millis();

	// Sharp curve left and right?
	bool any = false; // Any transistor senses dark (line)
	for (int8_t i = 0; i < 9; i++)
		if (mrm_ref_can->dark(i)) {
			if (i == 0 || i == 1)
				lastCurveRMs = millis();
			if (i == 7 || i == 8)
				lastCurveLMs = millis();
			any = true;
		}

	if (enteredCrossingAtMs != 0) {// Already in crossing
		if (millis() - lastCurveLMs < CROSSING_DURATION_MS && millis() - lastCurveRMs < CROSSING_DURATION_MS || (mrm_ref_can->dark(0) && mrm_ref_can->dark(1) && mrm_ref_can->dark(7) && mrm_ref_can->dark(8)))  // Still in crossing
			motorGroup->go(TOP_SPEED / 2, TOP_SPEED / 2);	// Go straight ahead slowly.
		else{ // Crossing over
			enteredCrossingAtMs = 0;
			if (lastGreenLeftMs > millis() - GREEN_BEFORE_MS)
				if (lastGreenRightMs > millis() - GREEN_BEFORE_MS) { // Turn backwards
					display(LED_CROSSING_BOTH_MARKS);
					turn(180);
				}
				else { // Turn left
					display(LED_CROSSING_MARK_LEFT);
					turn(-90);
				}
			else
				if (lastGreenRightMs > millis() - GREEN_BEFORE_MS) { // Turn right
					display(LED_CROSSING_MARK_RIGHT);
					turn(90);
				}
				else // Go ahead
					display(LED_CROSSING_NO_MARK);
		}
	}
	else if (millis() - lastCurveLMs < CURVE_BEFORE_MS && millis() - lastCurveRMs < CURVE_BEFORE_MS) { // Both edges sense line - crossing
		//print("STOP %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
		if (enteredCrossingAtMs == 0)
			enteredCrossingAtMs = millis();
		motorGroup->go(TOP_SPEED / 2, TOP_SPEED / 2);	// Go straight ahead slowly.
	}
	else if (any)  { 
		if ((mrm_ref_can->dark(3) || mrm_ref_can->dark(4) || mrm_ref_can->dark(5)) && (mrm_ref_can->dark(0) || mrm_ref_can->dark(8))) // Center and edge sensors - it could be a L turning or a crossing. Continue straight ahead. If turning, turn after losing the line.
			motorGroup->go(TOP_SPEED, TOP_SPEED);
		else {// Follow line. Maximum speed of the faster motor, decrease the other one.
			float lineCenter = (mrm_ref_can->center() - 5000) / 80.0; // Result: -50 to 50.
			motorGroup->go(lineCenter < 0 ? TOP_SPEED : TOP_SPEED - lineCenter * 3, lineCenter < 0 ? TOP_SPEED + lineCenter * 3 : TOP_SPEED);
			//lastPrinted = 0;
			interruptStartedMs = 0;
			if (lastGreenLeftMs > millis() - GREEN_BEFORE_MS)
				if (lastGreenRightMs > millis() - GREEN_BEFORE_MS)
					display(LED_LINE_FULL_BOTH_MARKS);
				else
					display(LED_LINE_FULL_MARK_LEFT);
			else
				if (lastGreenRightMs > millis() - GREEN_BEFORE_MS)
					display(LED_LINE_FULL_MARK_RIGHT);
				else
					display(LED_LINE_FULL);
		}
	}
	else {// No line 
		if (interruptStartedMs == 0)
			interruptStartedMs = millis();
		else if (millis() - lastCurveLMs < CURVE_BEFORE_MS) { // Last sharp curve was left
			//if (lastPrinted != 1)
			//	print("Left %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
			//lastPrinted = 1;
			motorGroup->go(-TOP_SPEED, TOP_SPEED);	// Rotate in place.
			lastCurveLMs = millis();
			display(LED_CURVE_LEFT);
		}
		else if (millis() - lastCurveRMs < CURVE_BEFORE_MS) { // Last sharp curve was right
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
				delay(1);
				mrm_col_can->illumination(0xFF, 0);
				delay(110);
			}
		}
	}
}

/** Avoid an obstacle on line
*/
void RobotLine::obstacleAvoid() {
	static uint8_t part = 0;
	static uint32_t startMs = 0;
	if (actionPreprocessing(true))
		display(LED_OBSTACLE);

	switch (part) {
	case 0: // Turn in place
		if (mrm_lid_can_b->reading(0) < 60 || mrm_ref_can->any(true))
			motorGroup->go(actionObstacleAvoid->leftOfObstacle ? -100 : 100, actionObstacleAvoid->leftOfObstacle ? 100 : -100);
		else {
			part = 1;
			startMs = millis();
		}
		break;
	case 1: // Continue turning even more
		if (millis() - startMs > 50) {
			part = 2;
			display(actionObstacleAvoid->leftOfObstacle ? LED_OBSTACLE_AROUND_LEFT : LED_OBSTACLE_AROUND_RIGHT);
		}
		break;
	case 2: // Go around obstacle
		if (mrm_ref_can->dark(4) || mrm_ref_can->dark(5)) { // Line found again
			motorGroup->go(actionObstacleAvoid->leftOfObstacle ? -100 : 100, actionObstacleAvoid->leftOfObstacle ? 100 : -100); // Start aligning with the line
			part = 3;
		}
		else
			motorGroup->go(actionObstacleAvoid->leftOfObstacle ? 95 : 30, actionObstacleAvoid->leftOfObstacle ? 30 : 95);
		break;
	case 3: // Align with the found line
		if (mrm_ref_can->dark(actionObstacleAvoid->leftOfObstacle ? 1 : 7))
			part = 4;
		break;
	case 4: // Follow line again
	default:
		part = 0;
		actionSet(actionLineFollow);
		actionPreprocessingEnd();
		break;
	}
}

/** Test for Mecanum wheels.
*/
void RobotLine::omniWheelsTest() {
	static uint8_t nextMove;
	static uint32_t lastMs;
	if (actionPreprocessing(true)) {
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

/** Starts the robot after this action selected.
*/
void RobotLine::rcjLine() {
	mrm_8x8a->rotationSet(LED_8X8_BY_90_DEGREES);
	bitmapsSet(); 
	display(LED_PLAY);
	devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.
	mrm_col_can->illumination(0xFF, 1);
	delayMs(20);
	actionSet(actionLineFollow);
}

/** Turns the robot clockwise using compass
*/
void RobotLine::turn(int16_t byDegreesClockwise) {
	motorGroup->go(TOP_SPEED, TOP_SPEED);
	delayMs(200);
	int16_t endAngle = mrm_imu->heading() + byDegreesClockwise;
	if (endAngle > 360)
		endAngle -= 360;
	else if (endAngle < 0)
		endAngle += 360;
	int8_t speed = byDegreesClockwise > 0 ? TOP_SPEED : -TOP_SPEED;
	motorGroup->go(speed, -speed);
	while (abs(mrm_imu->heading() - endAngle) > 5)
		noLoopWithoutThis();
	motorGroup->stop();
}

void RobotLine::wallFollow() {
	static uint32_t ms = 0;
	if (actionPreprocessing(true)) { // Only in the first pass.
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
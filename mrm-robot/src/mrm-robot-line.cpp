#include <mrm-8x8a.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-line.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotLine::RobotLine(char name[]) : Robot(name) {
	// MotorGroup class drives the motors.
	// 2nd, 4th, 6th, and 8th parameters are output connectors of the controller (0 - 3, meaning 1 - 4. connector). 2nd one must be connected to LB (Left-Back) motor,
	// 4th to LF (Left-Front), 6th to RF (Right-Front), and 8th to RB (Right-Back). Therefore, You can connect motors freely, but have to
	// adjust the parameters here. In this example output (connector) 3 is LB, etc.
	motorGroup = new MotorGroupDifferential(this, mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 3);

	// All the actions will be defined here; the objects will be created.
	actionEvacuationZone = new ActionEvacuationZone(this);
	actionLineFollow = new ActionLineFollow(this);
	actionObstacleAvoid = new ActionObstacleAvoid(this);
	actionRCJLine = new ActionRCJLine(this);
	actionWallFollow = new ActionWallFollow(this);
	actionStop = new ActionStop(this);

	// Generic actions
	actionLoopMenu = new ActionLoopMenu(this);
	actionLoop0 = new ActionLoop0(this);
	actionLoop1 = new ActionLoop1(this);
	actionLoop2 = new ActionLoop2(this);
	actionLoop3 = new ActionLoop3(this);
	actionLoop4 = new ActionLoop4(this);
	actionLoop5 = new ActionLoop5(this);
	actionLoop6 = new ActionLoop6(this);
	actionLoop7 = new ActionLoop7(this);
	actionLoop8 = new ActionLoop8(this);
	actionLoop9 = new ActionLoop9(this);

	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus.
	actionAdd(actionEvacuationZone);
	actionAdd(actionLineFollow);
	actionAdd(actionObstacleAvoid);
	actionAdd(actionRCJLine);
	actionAdd(actionWallFollow);

	// Generic actions
	actionAdd(actionLoopMenu);
	actionAdd(actionLoop0);
	actionAdd(actionLoop1);
	actionAdd(actionLoop2);
	actionAdd(actionLoop3);
	actionAdd(actionLoop4);
	actionAdd(actionLoop5);
	actionAdd(actionLoop6);
	actionAdd(actionLoop7);
	actionAdd(actionLoop8);
	actionAdd(actionLoop9);

	// Set buttons' actions.
	mrm_8x8a->actionSet(actionRCJLine, 0); // Button 0 starts RCJ Line.
	//mrm_8x8a->actionSet(actionEvacuationZone, 1); // Button 1 starts robot in evacution zone.
	//mrm_8x8a->actionSet(actionStop, 3);

	// Put Your buttons' actions here.

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. 
	mrm_mot4x3_6can->directionChange(0); // Uncomment to change 1st wheel's rotation direction
	mrm_mot4x3_6can->directionChange(1); // Uncomment to change 2nd wheel's rotation direction
	//mrm_mot4x3_6can->directionChange(2); // Uncomment to change 3rd wheel's rotation direction
	//mrm_mot4x3_6can->directionChange(3); // Uncomment to change 4th wheel's rotation direction

}

/** Arm will go to ball-catch position.
*/
void RobotLine::armCatch() {
	mrm_servo->write(LIFT_SERVO_DOWN, 0); // Lower the arm. Parameter 0 defines servo, in this case lift-servo. LIFT_SERVO_DOWN is angle.
	mrm_servo->write(CATCH_SERVO_CLOSE, 1); // Catch the ball. Parameter 1 - catch-servo. CATCH_SERVO_CLOSE is angle.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_DOWN, 3); // Rotate down.
}

/** Arm will go to ball-catch ready position.
*/
void RobotLine::armCatchReady() {
	mrm_servo->write(LIFT_SERVO_DOWN, 0); // Lower the arm.
	mrm_servo->write(CATCH_SERVO_OPEN_FULL, 1); // Open jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_DOWN, 3); // Rotate down.
}

/** Arm will go to idle (top) position.
*/
void RobotLine::armIdle() {
	mrm_servo->write(LIFT_SERVO_UP, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_OPEN_FULL, 1); // Open jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_DOWN, 3); // Rotate down.
} 

/** Arm will put the ball left
*/
void RobotLine::armLeftPut() {
	mrm_servo->write(LIFT_SERVO_PUT_BACK, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_OPEN_MIN, 1); // Open jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_LEFT, 3); // Rotate left.
}

/** Arm will go to top left position
*/
void RobotLine::armLeftReady() {
	mrm_servo->write(LIFT_SERVO_BACK, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_CLOSE, 1); // Close jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_LEFT, 3); // Rotate left.
}

/** Arm will drop the ball.
*/
void RobotLine::armPut() {
	mrm_servo->write(LIFT_SERVO_PUT_FRONT, 0); // Lift the arm halfways.
	mrm_servo->write(CATCH_SERVO_OPEN_FULL, 1); // Open jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
}

/** Arm will lift the caught ball in the position where will be ready to drop it.
*/
void RobotLine::armPutReady() {
	mrm_servo->write(LIFT_SERVO_PUT_FRONT, 0); // Lift the arm halfways.
	mrm_servo->write(CATCH_SERVO_CLOSE, 1); // Keep the ball in jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
}

/** Arm will put the ball right
*/
void RobotLine::armRightPut() {
	mrm_servo->write(LIFT_SERVO_PUT_BACK, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_OPEN_MIN, 1); // Open jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_RIGHT, 3); // Rotate right.
}

/** Arm will go to top right position
*/
void RobotLine::armRightReady() {
	mrm_servo->write(LIFT_SERVO_BACK, 0); // Lift the arm.
	mrm_servo->write(CATCH_SERVO_CLOSE, 1); // Close jaws.
	mrm_servo->write(BLOCK_SERVO_BOTH, 2); // Block both.
	mrm_servo->write(ROTATE_SERVO_RIGHT, 3); // Rotate right.
}

/** Barrier interrupted?
* return interrupted or not
*/
bool RobotLine::barrier() {
	return analogRead(35) < 2000; // 2000 is an example. Test to find Your best value
}

/** Stores bitmaps in mrm-led8x8a.
*/
void RobotLine::bitmapsSet() {
	mrm_8x8a->alive(0, true); // Makes sure that mrm-8x8a is present and functioning. If not, issues a warning message.

	// The 2 arrays will hold red and green pixels. Both red an green pixel on - orange color.
	uint8_t red[8];
	uint8_t green[8];

	/* 1 will turn the pixel on, 0 off. 0bxxxxxxxx is a binary format of the number. Start with "0b" and list all the bits, starting from
	the most significant one (MSB). Do that for each byte of the green and red arrays.*/

	// Evacuation zone
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	green[0] = 0b11111111;
	green[1] = 0b10000001;
	green[2] = 0b10000001;
	green[3] = 0b10000001;
	green[4] = 0b10000001;
	green[5] = 0b11000001;
	green[6] = 0b11100001;
	green[7] = 0b11111111;
	mrm_8x8a->bitmapCustomStore(red, green, LED_EVACUATION_ZONE);

	// Full line, no marks
	for (uint8_t i = 0; i < 8; i++)
		green[i] = 0b00011000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL);

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
	/* Store this bitmap in mrm-8x8a. The 3rd parameter is bitmap's address. If You want to define new bitmaps, expand LedSign enum with
	Your names, and use the new values for Your bitmaps. This parameter can be a plain number, but enum keeps thing tidy.*/
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL_BOTH_MARKS);

	// Full line, left mark.
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

	// Full line, right mark.
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

	// Full crossing, both marks.
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
	mrm_8x8a->bitmapCustomStore(red, green, LED_FULL_CROSSING_BOTH_MARKS);

	// Full crossing, mark left.
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
	mrm_8x8a->bitmapCustomStore(red, green, LED_FULL_CROSSING_MARK_LEFT);

	// Full crossing, mark right.
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
	mrm_8x8a->bitmapCustomStore(red, green, LED_FULL_CROSSING_MARK_RIGHT);

	// Full crossing, no marks.
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b11111111;
	green[3] = 0b11111111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_FULL_CROSSING_NO_MARK);

	// Half crossing, mark right.
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b00011111;
	green[3] = 0b00011111;
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
	mrm_8x8a->bitmapCustomStore(red, green, LED_HALF_CROSSING_MARK_RIGHT);

	// Half crossing, mark left.
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b11111000;
	green[3] = 0b11111000;
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
	mrm_8x8a->bitmapCustomStore(red, green, LED_HALF_CROSSING_MARK_LEFT);

	// Half crossing right, no mark.
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b00011111;
	green[3] = 0b00011111;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;

	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b00000000;
	red[5] = 0b00000000;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_HALF_CROSSING_RIGHT_NO_MARK);

	// Half crossing left, no mark
	green[0] = 0b00011000;
	green[1] = 0b00011000;
	green[2] = 0b11111000;
	green[3] = 0b11111000;
	green[4] = 0b00011000;
	green[5] = 0b00011000;
	green[6] = 0b00011000;
	green[7] = 0b00011000;

	red[0] = 0b00000000;
	red[1] = 0b00000000;
	red[2] = 0b00000000;
	red[3] = 0b00000000;
	red[4] = 0b00000000;
	red[5] = 0b00000000;
	red[6] = 0b00000000;
	red[7] = 0b00000000;
	mrm_8x8a->bitmapCustomStore(red, green, LED_HALF_CROSSING_LEFT_NO_MARK);

	// Interrupted line.
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

	// Curve left.
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

	// Curve right.
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

	// Obstacle.
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

	// Around obstacle left.
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

	// Around obstacle right.
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

	// Pause.
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

	// Play.
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

	// T-crossing approached by left side.
	green[0] = 0b0100000;
	green[1] = 0b0100000;
	green[2] = 0b0100000;
	green[3] = 0b0111111;
	green[4] = 0b0100000;
	green[5] = 0b0100000;
	green[6] = 0b0100000;
	green[7] = 0b0100000;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_T_CROSSING_BY_L);

	// T-crossing approached by right side.
	green[0] = 0b0000100;
	green[1] = 0b0000100;
	green[2] = 0b0000100;
	green[3] = 0b1111100;
	green[4] = 0b0000100;
	green[5] = 0b0000100;
	green[6] = 0b0000100;
	green[7] = 0b0000100;
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0;
	mrm_8x8a->bitmapCustomStore(red, green, LED_T_CROSSING_BY_R);

	// Define Your bitmaps here.

}

/** Go through a curve
*/
void RobotLine::curve() {
	bool left = mrm_ref_can->dark(LAST_TRANSISTOR); // Otherwise right
	mrm_8x8a->bitmapCustomStoredDisplay(left ? LED_CURVE_LEFT : LED_CURVE_RIGHT); 

	// If a marker found, rotate and do not continue executing the rest of the function.
	if (markers() && (mrm_ref_can->dark(3) || mrm_ref_can->dark(4) || (LAST_TRANSISTOR == 8 && mrm_ref_can->dark(5)))) // Center line needed, too.
		return;

	// Go a little ahead to check if it is T-crossing approached by side. Also, take a better position for turning.
	motorGroup->go(TOP_SPEED, TOP_SPEED);
	uint32_t startMs = millis();
	while (millis() - startMs < AHEAD_IN_CROSSING * 0.65) {
		if (mrm_ref_can->dark(left ? 0 : LAST_TRANSISTOR))// Opposite sensor - full crossing
			return; // Exit curve handling as crossing is found.
		noLoopWithoutThis();
	}

	// Rotate
	delayMs(30);
	if (!mrm_ref_can->any(true, 0, 2, 5)) { // L curve, not T approached by side. Because, if T approached by side, center transistors would still sense black.
		motorGroup->stop();
		print("CURVE\n\r"); // AAA
		delayMs(2000);
		motorGroup->go(left ? -TOP_SPEED : TOP_SPEED, left ? TOP_SPEED : -TOP_SPEED); // Start rotating.
		startMs = millis();
		while (millis() - startMs < 1300) { // Rotate untill center line found, unless timeout expires.
			if (mrm_ref_can->dark(LAST_TRANSISTOR / 2)) { // Line in center?
				delayMs(150); // Align better
				return;
			}
			noLoopWithoutThis();
		}
	}
	else {
		motorGroup->stop();
		print("HALF CROSS\n\r"); //AAA
		delayMs(2000);
	}
}

/** Dark surface?
* return dark or not
*/
bool RobotLine::dark() {
	return analogRead(36) < 2000; // 2000 is an example. Test to find Your best value
}

/** Enter evacuation-zone algorithm
*/
void RobotLine::evacuationZone() {
	motorGroup->stop();
	actionEnd();
	//// This function is not finished. It just catches and drops a ball.
	//if (actionPreprocessing(true)) {
	//	devicesStart(1);
	//	print("Catch ready\n\r");
	//	armCatchReady();
	//	delayMs(500);
	//	motorGroup->go(50, 50);
	//}
	////print("Distance %i\n\r", mrm_lid_can_b->reading(1));
	//if (mrm_lid_can_b->reading(1) < 40) {
	//	print("Found");
	//	motorGroup->stop();
	//	armCatch();
	//	delay(1000);
	//	armPutReady();
	//	motorGroup->go(50, 50);
	//	delayMs(1000);
	//	motorGroup->stop();
	//	armPut();
	//	delay(500);
	//	armIdle();
	//}
}

/** Generic actions, use them as templates
*/
void RobotLine::loop0() {}
void RobotLine::loop1() {}
void RobotLine::loop2() {}
void RobotLine::loop3() {}
void RobotLine::loop4() {}
void RobotLine::loop5() {}
void RobotLine::loop6() {}
void RobotLine::loop7() {}
void RobotLine::loop8() {}
void RobotLine::loop9() {}

/** Generic menu
*/
void RobotLine::loopMenu() {
	menuLevel = 8;
	actionEnd();
}

/** Test - go straight ahead using a defined speed.
*/
void RobotLine::goAhead() {
	const uint8_t speed = 40;
	motorGroup->go(speed, speed);
	actionEnd(); // This command will cancel actions and the robot will return in the default idle loop, after displaying menu.
}

/** Follow a RCJ line.
*/
void RobotLine::lineFollow() {
	static uint32_t lastLineFoundMs = millis(); // Used to measure gap in line.

	// Obstacle?
	if (mrm_lid_can_b->reading(1) < 50 && mrm_lid_can_b->reading(1) != 0) { // Front sensor (1).
		//print("Obstacle: %i\n\r", mrm_lid_can_b->reading(1)); // For debugging.
		motorGroup->stop(); // Stop.
		delayMs(50); // Wait a little before another measurement to be sure that the lidar has enough time to send a fresh measurement.
		if (mrm_lid_can_b->reading(1) < 50) { // Check one more time. If detected again, this will be an obstacle.
			actionSet(actionObstacleAvoid); // Sets the new action.
			actionObstacleAvoid->leftOfObstacle = (mrm_lid_can_b->reading(0) > mrm_lid_can_b->reading(2)); // Decides if go left or right, depending on space available.
			return;
		}
	}

	// Line found?
	if (mrm_ref_can->any(true)) {
		// Both edge sensors and middle sensor? Crossing. Check markers.
		if (mrm_ref_can->dark(0) && mrm_ref_can->dark(LAST_TRANSISTOR / 2) && mrm_ref_can->dark(LAST_TRANSISTOR)) {
			// Green markers?
			if (!markers()) {// No mark detected. Go straight ahead.
				mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_NO_MARK); // Show sign.
				//delayMs(1000);
				motorGroup->go(TOP_SPEED, TOP_SPEED);
				delayMs(AHEAD_IN_CROSSING); // Make sure to cross the crossing.
			}
		}
		// Edge sensor? If so, sharp bending.
		else if (mrm_ref_can->dark(0) || mrm_ref_can->dark(LAST_TRANSISTOR)) {
			curve();
		}
		else {
			// Follow line
			float lineCenter = (mrm_ref_can->center() - 5000) / 80.0; // mrm-ref-can returns center of line. After the calculation, the result will be between -50 and 50.
			// Calculate slower motor's speed. The other one will run at top speed.
			motorGroup->go(lineCenter < 0 ? TOP_SPEED : TOP_SPEED - lineCenter * 4, lineCenter < 0 ? TOP_SPEED + lineCenter * 4 : TOP_SPEED);
			mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_FULL);
		}
		lastLineFoundMs = millis(); // Mark last time line detected.
	}
	// No line found for a long time -> evacuation area.
	else if (millis() - lastLineFoundMs > BIGGEST_GAP_IN_LINE_MS) {
		actionSet(actionEvacuationZone);
		mrm_8x8a->bitmapCustomStoredDisplay(LED_EVACUATION_ZONE);
		motorGroup->stop();
	}
	// No line found for s short time -> gap in line, continue straight ahead.
	else {
		motorGroup->go(TOP_SPEED, TOP_SPEED);
		mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_INTERRUPTED);
	}
}

/** Custom test. The function will be called many times during the test, till You issue "x" menu command.
*/
void RobotLine::loop() {
	if (setup())
		print("Setup");
	if (mrm_8x8a->switchRead(0, 0))
		end();
}

/** Check markers and turn if any found
@return - true if marker found, false otherwise
*/
bool RobotLine::markers() {
	motorGroup->stop(); // Stop and wait to be sure the color sensor read new values.
	delayMs(200);
	//bool greenLeft = mrm_col_can->patternRecognizedBy6Colors(0) == 2; // This function returns laerned pattern's number, for sensor 0 (left). Learned pattern 2 is green.
	//bool greenRight = mrm_col_can->patternRecognizedBy6Colors(1) == 2; // This function returns laerned pattern's number, for sensor 0 (left). Learned pattern 2 is green.
	bool greenLeft = mrm_col_can->value(0) < 70 && mrm_col_can->hue(0) > 60 && mrm_col_can->hue(0) < 70;
	bool greenRight = mrm_col_can->value(1) < 70 && mrm_col_can->hue(1) > 60 && mrm_col_can->hue(1) < 70;

	bool fullLineL = !mrm_ref_can->any(false, 0, 4, 7);
	bool fullLineR = !mrm_ref_can->any(false, 0, 0, 3);

	if (!fullLineL)
		greenLeft = false;
	if (!fullLineR)
		greenRight = false;

	bool found = true;
	if (greenLeft && greenRight) { // Both markers detected. Turn backward.
		mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_BOTH_MARKS); // Show sign.
		print("MARK ");
		surfacePrint(false, 3000);
		print(" MARK\n\r");
		turn(180); // Turn by 180º.
	}
	else if (greenLeft && !greenRight) { // Only left marker. Turn left.
		mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_MARK_LEFT); // Show sign.
		print("MARK ");
		surfacePrint(true, 3000);
		turn(-90); // Turn by 90º left.
	}
	else if (!greenLeft && greenRight) { //Only right marker. Turn right.
		mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_MARK_RIGHT); // Show sign.
		surfacePrint(false, 3000);
		print(" MARK\n\r");
		turn(90); // Turn by 90º right.
	}
	else {
		found = false;
		surfacePrint(true, 3000);
	}
	return found;
}

/** Avoid an obstacle on line.
*/
void RobotLine::obstacleAvoid() {
	// Static variables - their value will be retained between this function's calls, just like with global variables, but they have local scope.
	static uint8_t part = 0;
	static uint32_t startMs = 0;

	// This function will be executed many times during obstacle avoiding action, but only the first time the action (ActionObstacleAvoid) is executed will
	// "actionPreprocessing(true)" be true, thus executing the next instruction (showing a sign).
	if (actionPreprocessing(true))
		mrm_8x8a->bitmapCustomStoredDisplay(LED_OBSTACLE); // Show a sign.

	/* Obstacle evasive maneuver is an action big enough to justify its own ActionBase derived object, ActionObstacleAvoid, which is being executed right now. 
	The maneuver itself consists of	different phases. Should all of them form ActionObstacleAvoid derived classes? Yes, that would be a good solution. However, 
	here we will show a solution that is not object oriented: a "switch" statement. During many runs of this function, static variable "part" will be slowly 
	changing its value from 0 up, marking different phases. The logic follows.
	*/
	switch (part) {
	case 0: // Turn in place in front of obstacle.
		if (mrm_lid_can_b->reading(1) < 60 || mrm_ref_can->any(true)) // If obstacle is still in front or robot still on line - continue rotating.
			// Read the member variable of the current action (of class ActionObstacleAvoid) to determine direction of rotation.
			motorGroup->go(actionObstacleAvoid->leftOfObstacle ? -100 : 100, actionObstacleAvoid->leftOfObstacle ? 100 : -100);
		else { // No more obastacle or line.
			part = 1; // Advance to next phase.
			startMs = millis(); // Mark phase's beginning.
		}
		break;
	case 1: // Continue turning even more.
		if (millis() - startMs > 50) { // A fixed number of milliseconds (50) is not the best solution, but it is here for demonstration purposes.
			part = 2; // Rotation over, advance to next phase.
			mrm_8x8a->bitmapCustomStoredDisplay(actionObstacleAvoid->leftOfObstacle ? LED_OBSTACLE_AROUND_LEFT : LED_OBSTACLE_AROUND_RIGHT); // Show sign.
		}
		break;
	case 2: // Go around obstacle
		// One of 2 middle sensors found a line?
		if (mrm_ref_can->dark(4) || mrm_ref_can->dark(5)) { // Yes, the line found again.
			motorGroup->go(actionObstacleAvoid->leftOfObstacle ? -100 : 100, actionObstacleAvoid->leftOfObstacle ? 100 : -100); // Start aligning with the line.
			part = 3; // Advance to next phase.
		}
		// No line found, continue going around the obstacle.
		else
			// Here the fixed motors' speed are used - and that is not good. You should improve this action by feeding some sensors' output back into this motor-driving loop.
			motorGroup->go(actionObstacleAvoid->leftOfObstacle ? 95 : 30, actionObstacleAvoid->leftOfObstacle ? 30 : 95);
		break;
	case 3: // Align with the found line. To do that, continue rotating till an near-edge sensor finds the line.
		if (mrm_ref_can->dark(actionObstacleAvoid->leftOfObstacle ? 1 : 7))
			part = 4; // Advance to next phase.
		break;
	case 4: // Follow line again.
	default:
		part = 0; // This is the last phase so reset "part" variable for the next obstacle.
		actionSet(actionLineFollow); // As the robot is now on line, set the current action to line following.
		actionPreprocessingEnd();
		break;
	}
}

/** Starts the RCJ Line run after this action selected.
*/
void RobotLine::rcjLine() {
	mrm_8x8a->rotationSet(LED_8X8_BY_90_DEGREES); // Rotate the mrm-8x8a by 90º so that it can be read properly when standing behind the robot.
	bitmapsSet(); // Upload all the predefined bitmaps into the mrm-8x8a.
	mrm_8x8a->bitmapCustomStoredDisplay(LED_PLAY); // Show "play" sign.
	mrm_col_can->illumination(0xFF, 1); // Turn mrm-col-can's surface illumination on.
	armIdle(); // Arm will go to its idle (up) position.
	actionSet(actionLineFollow); // The next action is line following.
}

/** Prints line and color sensors. Used for debugging.
@param newLine - new line
@param delayMsAfterPrint - delay after print
*/
void RobotLine::surfacePrint(bool newLine, uint16_t delayMsAfterPrint) {
	print("%i/%i/%i ", mrm_col_can->hue(0), mrm_col_can->saturation(0), mrm_col_can->value(0));
	for (uint8_t i = 0; i <= LAST_TRANSISTOR; i++)
		print("%i", mrm_ref_can->dark(i));
	print(" %i/%i/%i ", mrm_col_can->hue(1), mrm_col_can->saturation(1), mrm_col_can->value(1));
	if (newLine)
		print("\n\r");
	if (delayMsAfterPrint != 0)
		delayMs(delayMsAfterPrint);
}

/** Turns the robot clockwise using compass.
@param byDegreesClockwise - turn by defined number of degrees.
*/
void RobotLine::turn(int16_t byDegreesClockwise) {
	motorGroup->go(TOP_SPEED, TOP_SPEED); // Go ahead at top speed.
	delayMs(200); // Wait till the robot is in the position in which the line will hit the middle of robot.
	int16_t endAngle = mrm_imu->heading() + byDegreesClockwise; // Determine end angle.
	// If the result is outside the 0 -360º, correct.
	if (endAngle > 360)
		endAngle -= 360;
	else if (endAngle < 0)
		endAngle += 360;
	int8_t speed = byDegreesClockwise > 0 ? TOP_SPEED : -TOP_SPEED;
	motorGroup->go(speed, -speed); // Start turning
	// This is an example of a local loop and this is not a good solution, but is here as an example.
	while (abs(mrm_imu->heading() - endAngle) > 5) // As long as the error is 5º or more, continue rotating.
		noLoopWithoutThis(); // If a local loop is necessary, You will have to be invoking this function all the time.
	motorGroup->stop(); // Rotation over, stop.
}

/** Follows a wall.
*/
void RobotLine::wallFollow() {
	static uint32_t ms = 0; // Static variable - its value will be retained between this function's calls, just like a globa variable, but it has local scope.
	// This function will be executed many times during the wall-following action, but only the first time the action (ActionWallFollow) is executed will
	// "actionPreprocessing(true)" be true, thus executing the next instructions.
	if (actionPreprocessing(true)) { 
		mrm_8x8a->bitmapCustomStoredDisplay(LED_PLAY); // Show a sign. Actually, this sign is not defined yet, so it shows play-sign instead.
		devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.
	}

	// Debugging information.
	if (millis() - ms > 100)
		print("%i\n\r", mrm_lid_can_b->reading());

	// A very crude way of wall following.
	if (mrm_lid_can_b->reading() < 100) 
		motorGroup->go(20, 80);
	else
		motorGroup->go(80, 20);
}
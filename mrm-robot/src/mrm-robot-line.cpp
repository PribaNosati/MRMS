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
	actionGenericMenu = new ActionGenericMenu(this);
	actionGeneric0 = new ActionGeneric0(this);
	actionGeneric1 = new ActionGeneric1(this);
	actionGeneric2 = new ActionGeneric2(this);
	actionGeneric3 = new ActionGeneric3(this);
	actionGeneric4 = new ActionGeneric4(this);
	actionGeneric5 = new ActionGeneric5(this);
	actionGeneric6 = new ActionGeneric6(this);
	actionGeneric7 = new ActionGeneric7(this);
	actionGeneric8 = new ActionGeneric8(this);
	actionGeneric9 = new ActionGeneric9(this);

	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus.
	actionAdd(actionEvacuationZone);
	actionAdd(actionLineFollow);
	actionAdd(actionObstacleAvoid);
	actionAdd(actionRCJLine);
	actionAdd(actionWallFollow);

	// Generic actions
	actionAdd(actionGenericMenu);
	actionAdd(actionGeneric0);
	actionAdd(actionGeneric1);
	actionAdd(actionGeneric2);
	actionAdd(actionGeneric3);
	actionAdd(actionGeneric4);
	actionAdd(actionGeneric5);
	actionAdd(actionGeneric6);
	actionAdd(actionGeneric7);
	actionAdd(actionGeneric8);
	actionAdd(actionGeneric9);
	// Set buttons' actions.
	mrm_8x8a->actionSet(actionRCJLine, 0); // Button 0 starts RCJ Line.
	mrm_8x8a->actionSet(actionEvacuationZone, 1); // Button 1 starts robot in evacution zone.
	mrm_8x8a->actionSet(actionStop, 3);
	// Put Your buttons' actions here.

	// Depending on your wiring, it may be necessary to spin some motors in the other direction. 
	mrm_mot4x3_6can->directionChange(0); // Uncomment to change 1st wheel's rotation direction
	mrm_mot4x3_6can->directionChange(1); // Uncomment to change 2nd wheel's rotation direction
	//mrm_mot4x3_6can->directionChange(2); // Uncomment to change 3rd wheel's rotation direction
	//mrm_mot4x3_6can->directionChange(3); // Uncomment to change 4th wheel's rotation direction

}

/** Custom test. The function will be called many times during the test, till You issue "x" menu command.
*/
void RobotLine::anyTest() {
	static bool avoidingExit = false;

	if (actionPreprocessing(true))
		devicesStart(1);

	if (mrm_lid_can_b->reading(1) < avoidingExit ? 200 : 90) // Wall ahead?
		motorGroup->go(-50, 50);
	else {
		if (mrm_lid_can_b->reading(2) > 400) { // Exit?
			motorGroup->go(50, 50);
			avoidingExit = true;
		}
		else {
			int error = (mrm_lid_can_b->reading(2) - 100) * 0.5;
			error = constrain(error, -50, 50);
			motorGroup->go(50 + error, 50 - error);
			avoidingExit = false;
		}
	}
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
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0, green[i] = 0;

	/* 1 will turn the pixel on, 0 off. 0bxxxxxxxx is a binary format of the number. Start with "0b" and list all the bits, starting from
	the most significant one (MSB). Do that for each byte of the green and red arrays.*/

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
	/* Store this bitmap in mrm-8x8a. The 3rd parameter is bitmap's address. If You want to define new bitmaps, expand LedSign enum with
	Your names, and use the new values for Your bitmaps. This parameter can be a plain number, but enum keeps thing tidy.*/
	mrm_8x8a->bitmapCustomStore(red, green, LED_LINE_FULL_BOTH_MARKS);
	delayMs(1); // Wait a little in order not to queue too many CAN Buss messages at once.

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

	// Full crossing, no marks.
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
	mrm_8x8a->bitmapCustomStore(red, green, LED_FULL_CROSSING_NO_MARK);
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

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
	delayMs(1);

	// Define Your bitmaps here.

	delayMs(10); // Wait a little to be sure that the next sign will be displayed.
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
	// This function is not finished. It just catches and drops a ball.
	if (actionPreprocessing(true)) {
		devicesStart(1);
		print("Catch ready\n\r");
		armCatchReady();
		delayMs(500);
		motorGroup->go(50, 50);
	}
	//print("Distance %i\n\r", mrm_lid_can_b->reading(1));
	if (mrm_lid_can_b->reading(1) < 40) {
		print("Found");
		motorGroup->stop();
		armCatch();
		delay(1000);
		armPutReady();
		motorGroup->go(50, 50);
		delayMs(1000);
		motorGroup->stop();
		armPut();
		delay(500);
		armIdle();
	}
}

/** Generic actions, use them as templates
*/
void RobotLine::generic0() {}
void RobotLine::generic1() {}
void RobotLine::generic2() {}
void RobotLine::generic3() {}
void RobotLine::generic4() {}
void RobotLine::generic5() {}
void RobotLine::generic6() {}
void RobotLine::generic7() {}
void RobotLine::generic8() {}
void RobotLine::generic9() {}

/** Generic menu
*/
void RobotLine::genericMenu() {
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

	// After the robot loses the line, it will check when it detected it with far left or far right sensors. If it occured within last CURVE_BEFORE_MS ms, it will 
	// conclude that it was a sharp (L-type) turning) and will turn to that side to catch the lost line again. 100 ms for speed 90.
	const uint16_t CURVE_BEFORE_MS = 200; 
	// If far left sensor detected a line at most before CROSSING_DURATION_MS ms, and the same for the far right sensor, this will be a crossing.
	const uint16_t CROSSING_DURATION_MS = 100;
	// When checking if a green mark was detected in crossing, it checks if the detection occured at most before GREEN_BEFORE_MS ms. If so, a green mark found.
	const uint16_t GREEN_BEFORE_MS = 400;
	// This parameter determines turning vigour. The higher it is, the more the robot will turn. A too big value will cause oscillations.
	const uint8_t TURNING_STRENGTH = 4;

	// Static variables - their value will be retained between this function's calls, just like with global variables, but they have local scope.
	static uint32_t interruptStartedMs = 0;
	static uint8_t lastPrinted = 0;
	static uint32_t lastCurveLMs = 0;
	static uint32_t lastCurveRMs = 0;
	static uint32_t lastGreenLeftMs = 0;
	static uint32_t lastGreenRightMs = 0;
	static uint32_t enteredCrossingAtMs = 0;
	static uint32_t ms = 0;

	// Obstacle?
	if (mrm_lid_can_b->reading(1) < 50 && mrm_lid_can_b->reading(1) != 0) { // Front sensor (1).
		print("Obstacle: %i\n\r", mrm_lid_can_b->reading(1)); // For debugging.
		motorGroup->stop(); // Stop.
		delayMs(50); // Wait a little before another measurement to be sure that the lidar has enough time to send a fresh measurement.
		if (mrm_lid_can_b->reading(1) < 50) { // Check one more time. If detected again, this will be an obstacle.
			actionSet(actionObstacleAvoid); // Sets the new action.
			actionObstacleAvoid->leftOfObstacle = (mrm_lid_can_b->reading(0) > mrm_lid_can_b->reading(2)); // Decides if go left or right, depending on space available.
			return;
		}
	}

	// Green?
	if (mrm_col_can->patternRecognizedBy6Colors(0) == 2) // This function returns laerned pattern's number, for sensor 0 (left). Learned pattern 2 is green.
		lastGreenLeftMs = millis(); // Marks the time. It will be needed later, if the robot detects a crossing.
	if (mrm_col_can->patternRecognizedBy6Colors(1) == 2) // This function returns laerned pattern's number, for sensor 0 (left). Learned pattern 2 is green.
		lastGreenRightMs = millis(); // Marks the time. It will be needed later, if the robot detects a crossing.

	// Sharp curve left or right?
	bool anyLine = false; // Any transistor senses dark (line).
	for (int8_t i = 0; i < 9; i++) // 9 transistors.
		if (mrm_ref_can->dark(i)) {
			if (i == 0) // Sensor 0 or 1 - curve right.
				lastCurveRMs = millis();
			if (i == 8) // 7 or 8 - left.
				lastCurveLMs = millis();
			anyLine = true;
		}

	// Follow line
	float lineCenter = (mrm_ref_can->center() - 5000) / 80.0; // mrm-ref-can returns center of line. After the calculation, the result will be between -50 and 50.
	// Calculate slower motor's speed. The other one will run at top speed.
	motorGroup->go(lineCenter < 0 ? TOP_SPEED : TOP_SPEED - lineCenter * 3, lineCenter < 0 ? TOP_SPEED + lineCenter * 3 : TOP_SPEED); 

	// L turn left or right?
	if ((mrm_ref_can->dark(0) || mrm_ref_can->dark(8)) && (mrm_ref_can->dark(3) || mrm_ref_can->dark(4) || mrm_ref_can->dark(5))) {
		if (!markers(lastGreenLeftMs, lastGreenRightMs, GREEN_BEFORE_MS))
			if (mrm_ref_can->dark(0) && mrm_ref_can->dark(8)) {
				motorGroup->go(TOP_SPEED, TOP_SPEED);
				delayMs(20);
			}
	}

	// Instead going straight ahead, started to turn?
	uint8_t phase = 0;
	for (uint8_t i = 0; i < 9 && phase != 4; i++) {
		switch (phase) {
		case 0: // 1 dark sensor
			if (mrm_ref_can->dark(i))
				phase = 1;
			break;
		case 1: // 2 white sensors
		case 2:
			if (!mrm_ref_can->dark(i))
				phase++;
			break;
		case 3: // 1 dark sensor
			if (mrm_ref_can->dark(i))
				phase = 4;
			break;
		}
	}
	if (phase == 4){
		turn(lastCurveLMs > lastCurveRMs ? 30 : -30);
		motorGroup->go(TOP_SPEED, TOP_SPEED);
		delayMs(20);
	}

	//// Already in full crossing?
	//if (enteredCrossingAtMs != 0) { // Yes, already in crossing.
	//	// If either left and right far sensors are detecting the line, it is still in crossing. Allow some time after crossing exit.
	//	if (mrm_ref_can->dark(0) && mrm_ref_can->dark(1) || mrm_ref_can->dark(7) && mrm_ref_can->dark(8) || 
	//		(millis() - lastCurveLMs < 50 || millis() - lastCurveRMs < 50) && (mrm_ref_can->dark(3) || mrm_ref_can->dark(4) || mrm_ref_can->dark(5)))
	//		motorGroup->go(TOP_SPEED / 2, TOP_SPEED / 2);	// Go straight ahead slowly.
	//	else{ // Crossing over as no edge sensors activated, time for decision what to do
	//		enteredCrossingAtMs = 0; // Mark end of crossing.
	//		// Full crossing?
	//		if (millis() - lastCurveLMs < CROSSING_DURATION_MS && millis() - lastCurveRMs < CROSSING_DURATION_MS) {
	//			// Depending on detected green marks, decide what to do.
	//			if (lastGreenLeftMs > millis() - GREEN_BEFORE_MS) // Left marker detected.
	//				if (lastGreenRightMs > millis() - GREEN_BEFORE_MS) { // The right detected, too. Turn backwards.
	//					mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_BOTH_MARKS); // Show sign.
	//					motorGroup->stop();
	//					print("180: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
	//					delayMs(5000);
	//					turn(180); // Turn by 180º.
	//				}
	//				else { // Only left marker detected. Turn left.
	//					mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_MARK_LEFT); // Show sign.
	//					motorGroup->stop();
	//					print("FULL L: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
	//					delayMs(5000);
	//					turn(-90); // Turn by 90º left.
	//				}
	//			else // Left marker not detected.
	//				if (lastGreenRightMs > millis() - GREEN_BEFORE_MS) { // Only right marker detected. Turn right.
	//					mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_MARK_RIGHT); // Show sign.
	//					motorGroup->stop();
	//					print("FULL R: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
	//					delayMs(5000);
	//					turn(90); // Turn by 90º right.
	//				}
	//				else {// No mark detected. Go straight ahead.
	//					mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_NO_MARK); // Show sign.
	//					motorGroup->stop();
	//					print("FULL AHE: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
	//					delayMs(5000);
	//					motorGroup->go(TOP_SPEED, TOP_SPEED);
	//				}
	//		}
	//	}
	//}
	//// Not in full crossing already but maybe just entered it?
	//else if (millis() - lastCurveLMs < CURVE_BEFORE_MS || millis() - lastCurveRMs < CURVE_BEFORE_MS) { // Both edge sensors sense a line - crossing.
	//	//print("STOP %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
	//	if (enteredCrossingAtMs == 0) // This should be 0.
	//		enteredCrossingAtMs = millis(); // Mark crossing start.
	//	motorGroup->go(TOP_SPEED / 2, TOP_SPEED / 2); // Go straight ahead slowly to faciliate green marks' detection.
	//}

	// No crossing. Is there a line detected at all?
	//else if (anyLine)  { // Yes, there is a line.
	//	// Center and edge sensors?
	//	if ((mrm_ref_can->dark(3) || mrm_ref_can->dark(4) || mrm_ref_can->dark(5)) && (mrm_ref_can->dark(0) || mrm_ref_can->dark(8))) 
	//		// Yes, center and edge sensors. It could be a L turning or a crossing. Continue straight ahead as it is not clear yet. Later, if turning, turn after losing the line.
	//		motorGroup->go(TOP_SPEED, TOP_SPEED);
	//	else {// Some sensors detected a line, but it doesn't look as a very sharp turning or a crossing. Just follow the line. Maximum speed of the faster motor, decrease the other one.
	//		float lineCenter = (mrm_ref_can->center() - 5000) / 80.0; // mrm-ref-can returns center of line. After the calculation, the result will be between -50 and 50.
	//		// Calculate slower motor's speed. The other one will run at top speed.
	//		motorGroup->go(lineCenter < 0 ? TOP_SPEED : TOP_SPEED - lineCenter * 3, lineCenter < 0 ? TOP_SPEED + lineCenter * 3 : TOP_SPEED); 
	//		interruptStartedMs = 0; // Remember that no line interrupt happened here.
	//		// Display detected green markers. Even when not in crossings, it is useful for debugging. False detections can occur when a sensor is partially over black and
	//		// partially over a white surface. Black and white can have any hue and saturation (so, green included). In this split black-white detection, value (in HSV) can also hit
	//		// green's value. In that case, a marker will be detected even there is none.
	//		if (lastGreenLeftMs > millis() - GREEN_BEFORE_MS)
	//			if (lastGreenRightMs > millis() - GREEN_BEFORE_MS)
	//				mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_FULL_BOTH_MARKS);
	//			else
	//				mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_FULL_MARK_LEFT);
	//		else
	//			if (lastGreenRightMs > millis() - GREEN_BEFORE_MS)
	//				mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_FULL_MARK_RIGHT);
	//			else
	//				mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_FULL);
	//	}
	//}
	//else {// No line.
	//	// Interrupt started now?
	//	if (interruptStartedMs == 0) // Yes
	//		interruptStartedMs = millis(); // Mark when the line disappeared.
	//	// Was there a sharp left curve lately?
	//	else if (millis() - lastCurveLMs < CURVE_BEFORE_MS) { // Yes.
	//		//if (lastPrinted != 1)
	//		//	print("Left %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
	//		//lastPrinted = 1;
	//		motorGroup->go(-TOP_SPEED, TOP_SPEED); // Rotate in place to catch the lost line left.
	//		lastCurveLMs = millis(); // Make sure that turning will continue.
	//		mrm_8x8a->bitmapCustomStoredDisplay(LED_CURVE_LEFT); // Show sign.
	//	}
	//	// No lost line left but maybe there was a sharp right curve lately.
	//	else if (millis() - lastCurveRMs < CURVE_BEFORE_MS) { // Yes.
	//		//if (lastPrinted != 2)
	//		//	print("Right %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
	//		//lastPrinted = 2;
	//		motorGroup->go(TOP_SPEED, -TOP_SPEED); // Rotate in place to catch the lost line right.
	//		lastCurveRMs = millis(); // Make sure that turning will continue.
	//		mrm_8x8a->bitmapCustomStoredDisplay(LED_CURVE_RIGHT); // Show sign.
	//	}
	//	// No sharp curve in the near past. Therefore, a straight line was interrupted.
	//	else {							
	//		//if (lastPrinted != 3)
	//		//	print("Ahead %i %i.\n\r", millis() - lastCurveLMs, millis() - lastCurveRMs);
	//		//lastPrinted = 3;
	//		motorGroup->go(TOP_SPEED, TOP_SPEED);  // Go straight ahead to overcome the gap.
	//		mrm_8x8a->bitmapCustomStoredDisplay(LED_LINE_INTERRUPTED); // Show sign.
	//		if (millis() - interruptStartedMs > 2000) { // If the interrupt lasted too long, end the run.
	//			mrm_8x8a->bitmapCustomStoredDisplay(LED_PAUSE); // Display sign.
	//			actionEnd(); // Stop the current action.
	//			motorGroup->stop(); // Stop the robot.
	//			delayMs(1);
	//			mrm_col_can->illumination(0xFF, 0); // Turn off color sensors' illumination.
	//			delayMs(110);
	//		}
	//	}
	//}
}

bool RobotLine::markers(uint32_t lastGreenLeftMs, uint32_t lastGreenRightMs, uint32_t greenLimitMs) {
	bool found = true;
	if (lastGreenLeftMs > millis() - greenLimitMs) // Left marker detected.
		if (lastGreenRightMs > millis() - greenLimitMs) { // The right detected, too. Turn backwards.
			mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_BOTH_MARKS); // Show sign.
			motorGroup->stop();
			print("180: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
			delayMs(5000);
			turn(180); // Turn by 180º.
		}
		else { // Only left marker detected. Turn left.
			mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_MARK_LEFT); // Show sign.
			motorGroup->stop();
			print("FULL L: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
			delayMs(5000);
			turn(-90); // Turn by 90º left.
		}
	else // Left marker not detected.
		if (lastGreenRightMs > millis() - greenLimitMs) { // Only right marker detected. Turn right.
			mrm_8x8a->bitmapCustomStoredDisplay(LED_FULL_CROSSING_MARK_RIGHT); // Show sign.
			motorGroup->stop();
			print("FULL R: %i %i \n\r", mrm_ref_can->dark(4), mrm_ref_can->dark(5));
			delayMs(5000);
			turn(90); // Turn by 90º right.
		}
		else {// No mark detected. 
			found = false;
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
	devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values (parameter 1 determines this action).
	mrm_col_can->illumination(0xFF, 1); // Turn mrm-col-can's surface illumination on.
	armIdle(); // Arm will go to its idle (up) position.
	delayMs(20); // Wait so that the next sign will definitely show.
	actionSet(actionLineFollow); // The next action is line following.
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
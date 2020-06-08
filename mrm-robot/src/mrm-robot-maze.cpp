#include <mrm-8x8a.h>
#include <mrm-col-can.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-maze.h"
#include <mrm-servo.h>
#include <mrm-therm-b-can.h>

Tile* Tile::last; // Neccessary for static member variable.

RobotMaze::RobotMaze() : Robot() {
	motorGroup = new MotorGroupDifferential(mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 3);

	actionDecide = new ActionDecide(this);
	actionGoStraightAhead = new ActionMoveAhead(this);
	actionMap = new ActionMap(this);
	actionMove = new ActionMove(this);
	actionMoveAhead = new ActionMoveAhead(this);
	actionMoveTurn = new ActionMoveTurn(this);

	actionAdd(new ActionOmniWheelsTest(this));
}

/** Custom test
*/
void RobotMaze::anyTest() {
	static bool yes = true;
	if (actionPreprocessing(true)) {
		mrm_lid_can_b->reset();
		delay(1);
	}
	actionEnd();
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotMaze::bitmapsSet() {
	mrm_8x8a->alive(0, true);
	uint8_t red[8];
	uint8_t green[8];
	for (uint8_t i = 0; i < 8; i++)
		red[i] = 0, green[i] = 0;

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
	mrm_8x8a->bitmapCustomStore(red, green, MAZE_LED_PAUSE);
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
	mrm_8x8a->bitmapCustomStore(red, green, MAZE_LED_PLAY);
	delayMs(10);

}


/**Tremaux
*/
void RobotMaze::decide(){
	directionToGo = NOWHERE;
	for (uint8_t i = 0; i < 4; i++) {
		Direction dir = (Direction)i;
		if (tileCurrent->wallGet(dir) == NO_WALL) {
			directionToGo = dir;
			break;
		}
	}
	if (directionToGo == NOWHERE) // Breadcrumbs
		if (tileCurrent->breadcrumb == NOWHERE) //Start tile and no unvisited direction
			actionEnd();
		else
			directionToGo = tileCurrent->breadcrumb;
}

/** Displays a bitmap using mrm-8x8.
@param bitmap - user bitmap's id
*/
void RobotMaze::display(uint8_t bitmap) {
	static uint8_t lastDisplayeBitmap = 0xFF;
	static uint32_t lastMs = 0;
	if (bitmap != lastDisplayeBitmap && millis() - lastMs > 10) {
		mrm_8x8a->bitmapCustomStoredDisplay(bitmap);
		lastMs = millis();
		lastDisplayeBitmap = bitmap;
	}
}

void RobotMaze::imuFollow() {
	float errorCW = imuLastValid - mrm_imu->heading();
	int16_t slowerMotor = TOP_SPEED - errorCW * IMU_FOLLOW_STRENGTH;
	motorGroup->go(errorCW > 0 ? slowerMotor : TOP_SPEED, errorCW < 0 ? TOP_SPEED : slowerMotor);
}

void RobotMaze::map() {
	for (uint8_t i = 0; i < 4; i++) {
		Direction dir = (Direction)i;
		if (tileCurrent->wallGet(dir) == WALL_UNKNOWN)
			if (distance(dir, false) < NO_WALL_DISTANCE && distance(dir, true) < NO_WALL_DISTANCE) // Maybe "||" ?
				tileCurrent->wallSet(dir, WALL_WITHOUT_VICTIM);
	}
	actionSet(actionDecide);
}

void RobotMaze::move() {
	if (directionToGo != directionCurrent)
		actionSet(actionMoveTurn); // turn
	else {
		imuLastValid = 9999; // Reset direction. First wall-follow will set the correct one.
		actionSet(actionMoveAhead);// go straight
	}
}

void RobotMaze::moveAhead() {
	bool encodersOver = true;
	bool tileExists = true;
	bool wallInReach = true;

	if (encodersOver) {
		// Calculate new coordinates
		int8_t newX = tileCurrent->x;
		int8_t newY = tileCurrent->y;
		if (directionCurrent == UP)
			newY++;
		else if (directionCurrent == LEFT)
			newX--;
		else if (directionCurrent == DOWN)
			newY--;
		else
			newX++;
		Tile* targetTile = tileContaining(newX, newY);//Search chain for x, y
		if (targetTile != NULL) {
			tileCurrent = targetTile; // move 1 tile
			actionSet(actionDecide);
		}
		else {
			tileCurrent = new Tile(newX, newY, (Direction)((directionCurrent + 2) % 4)); // Opposite direction.
			actionSet(actionMap);
		}
	}
	else {
		Direction closestWall = wallClosest();
		if (closestWall != NOWHERE)
			wallFollow(closestWall);
		else
			imuFollow();
	}
}

void RobotMaze::moveTurn() {
	//turn();
	directionCurrent = directionToGo;
	actionSet(actionDecide);
}


/** Test for Mecanum wheels.
*/
void RobotMaze::omniWheelsTest() {
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

void RobotMaze::rescueMaze() {
	mrm_8x8a->rotationSet(LED_8X8_BY_90_DEGREES);
	bitmapsSet();
	display(MAZE_LED_PLAY);
	devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.
	tileStart = new Tile(0, 0, NOWHERE);
	Tile::last = NULL;
	tileCurrent = tileStart;
	directionCurrent = UP;
	actionSet(actionMap);
}

Tile* RobotMaze::tileContaining(int8_t x, int8_t y) {
	for (Tile* curr = tileStart; curr->next() != NULL; curr = curr->next())
		if (curr->x == x && curr->y == y)
			return curr;
	return NULL;
}

/** Turns the robot clockwise using compass
*/
void RobotMaze::turn(int16_t byDegreesClockwise) {
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

void RobotMaze::wallFollow(Direction wallDirection) {
	int16_t differenceCWMinusCCW = distance(wallDirection, 1) - distance(wallDirection, 0);
	int16_t slowerMotor = TOP_SPEED - differenceCWMinusCCW * WALL_FOLLOW_STRENGTH;
	if (directionCurrent = wallDirection) // Towards wall to follow
		motorGroup->go(slowerMotor, TOP_SPEED);
	else if ((directionCurrent + 2) % 4 == wallDirection) // Away from wall to follow
		motorGroup->go(TOP_SPEED, slowerMotor);
	else if (directionCurrent + 1 == wallDirection) // Wall to the left
		motorGroup->go(TOP_SPEED, slowerMotor);
	else // Wall to the right
		motorGroup->go(slowerMotor, TOP_SPEED);
	imuLastValid = mrm_imu->heading(); // Correct stored direction.
	//static uint32_t ms = 0;
	//if (actionPreprocessing(true)) { // Only in the first pass.
	//	display(LED_PLAY);
	//	devicesStart(1); // Commands all sensors to start sending measurements. mrm-ref-can will be sending digital values.
	//}

	//if (millis() - ms > 100)
	//	print("%i\n\r", mrm_lid_can_b->reading());

	//if (mrm_lid_can_b->reading() < 100) {
	//	motorGroup->go(20, 80);
	//}
	//else {
	//	motorGroup->go(80, 20);
	//}
}

Direction RobotMaze:: wallClosest() {
	Direction closest = NOWHERE;
	uint16_t leastDistance = 0xFFFF;
	for (uint8_t i = 0; i < 4; i++) {
		Direction dir = (Direction)i;
		uint16_t d0 = distance(dir, false);
		uint16_t d1 = distance(dir, true);
		if (d0 < NO_WALL_DISTANCE && d1 < NO_WALL_DISTANCE && abs(d0 - d1) < WALL_FOLLOW_ERROR_ALLOWED) {
			if (d0 < d1) d0 = d1;
			if (leastDistance > d0) {
				leastDistance = d0;
				closest = dir;
			}
		}
	}
	return closest;
}

Tile::Tile(int8_t xNow, int8_t yNow, Direction breadcrumbNow) {
	x = xNow;
	y = yNow;
	breadcrumb = breadcrumb;
	Tile::last->_chain = this;
	last = this;
}
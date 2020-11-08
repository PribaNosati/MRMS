#pragma once
#include <mrm-action.h>
#include <mrm-robot.h>

// Change these values to get optimal robot's behaviour.

// CATCH_SERVO drives jaws that catch ball.
#define CATCH_SERVO_CLOSE 250 // Closed position, ball caught.
#define CATCH_SERVO_OPEN_FULL 70 // Open position, ready to catch a ball.
#define CATCH_SERVO_OPEN_MIN 150 // Open just to drop the ball

// LIFT_SERVO lifts catch servo and the rest of mechanism.
#define LIFT_SERVO_BACK 70 // Top (idle) position.
#define LIFT_SERVO_DOWN 130 // Lowest position, catching a ball.
#define LIFT_SERVO_PUT_BACK 50
#define LIFT_SERVO_PUT_FRONT 80 // Middle position, for caught ball dropping.
#define LIFT_SERVO_UP 10 // Top (idle) position.

// ROTATE_SERVO rotates gripper
#define ROTATE_SERVO_DOWN 300
#define ROTATE_SERVO_LEFT 90
#define ROTATE_SERVO_RIGHT 125

// BLOCK_SERVO blocks balls
#define BLOCK_SERVO_BOTH 150

// mrm-8x8a display bitmaps.
enum ledSign {LED_EVACUATION_ZONE, LED_FULL_CROSSING_BOTH_MARKS, LED_FULL_CROSSING_MARK_LEFT, LED_FULL_CROSSING_MARK_RIGHT, LED_FULL_CROSSING_NO_MARK,
	LED_HALF_CROSSING_MARK_LEFT, LED_HALF_CROSSING_MARK_RIGHT, LED_HALF_CROSSING_LEFT_NO_MARK, LED_HALF_CROSSING_RIGHT_NO_MARK,
	LED_LINE_FULL, LED_LINE_FULL_BOTH_MARKS, LED_LINE_FULL_MARK_LEFT, LED_LINE_FULL_MARK_RIGHT, 
	LED_LINE_INTERRUPTED, LED_CURVE_LEFT, LED_CURVE_RIGHT, LED_OBSTACLE, LED_OBSTACLE_AROUND_LEFT, LED_OBSTACLE_AROUND_RIGHT, LED_PAUSE, LED_PLAY, LED_T_CROSSING_BY_L, LED_T_CROSSING_BY_R};

/* All the Action-classes have to be forward declared here (before RobotLine) as RobotLine declaration uses them. The other option would be
not to declare them here, but in that case Action-objects in RobotLine will have to be declared as ActionBase class, forcing downcast later in code, if
derived functions are used.*/
class ActionEvacuationZone;
class ActionObstacleAvoid;
class ActionLineFollow;
class ActionRCJLine;

class ActionGeneric0;
class ActionGeneric1;
class ActionGeneric2;
class ActionGeneric3;
class ActionGeneric4;
class ActionGeneric5;
class ActionGeneric6;
class ActionGeneric7;
class ActionGeneric8;
class ActionGeneric9;
class ActionGenericMenu;

/** Robot for RCJ Rescue Line, a class derived from the base Robot class.
*/
class RobotLine : public Robot {
	uint16_t BIGGEST_GAP_IN_LINE_MS = 1500;
	// Changing this parameter will cause major behaviour change. Limit value: 127.
	const uint8_t TOP_SPEED = 60;

	// Actions' declarations
	ActionEvacuationZone* actionEvacuationZone;
	ActionObstacleAvoid* actionObstacleAvoid;
	ActionLineFollow* actionLineFollow;
	ActionRCJLine* actionRCJLine;
	ActionBase* actionWallFollow;
	ActionStop* actionStop;

	// Generic actions
	ActionGeneric0* actionGeneric0;
	ActionGeneric1* actionGeneric1;
	ActionGeneric2* actionGeneric2;
	ActionGeneric3* actionGeneric3;
	ActionGeneric4* actionGeneric4;
	ActionGeneric5* actionGeneric5;
	ActionGeneric6* actionGeneric6;
	ActionGeneric7* actionGeneric7;
	ActionGeneric8* actionGeneric8;
	ActionGeneric9* actionGeneric9;
	ActionGenericMenu* actionGenericMenu;

	MotorGroupDifferential* motorGroup = NULL; // Class that conveys commands to motors.

public:
	/** Constructor
	@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
	*/
	RobotLine(char name[] = "RCJ Line 1"); // Maximum 15 characters

	/** Custom test
	*/
	void anyTest();

	/** Arm will go to ball-catch position.
	*/
	void armCatch();

	/** Arm will go to ball-catch ready position.
	*/
	void armCatchReady();

	/** Arm will go to idle (top) position.
	*/
	void armIdle();

	/** Arm will put the ball left
	*/
	void armLeftPut();

	/** Arm will go to top left position
	*/
	void armLeftReady();

	/** Arm will drop the ball.
	*/
	void armPut();

	/** Arm will lift the caught ball in the position where will be ready to drop it.
	*/
	void armPutReady();

	/** Arm will put the ball right
	*/
	void armRightPut();

	/** Arm will go to top right position
	*/
	void armRightReady();

	/** Barrier interrupted?
	* return interrupted or not
	*/
	bool barrier();

	/** Stores bitmaps in mrm-led8x8a.
	*/
	void bitmapsSet();

	/** Dark surface?
	* return dark or not
	*/
	bool dark();

	/** Enter evacuation-zone algorithm.
	*/
	void evacuationZone();

	/** Generic actions, use them as templates
	*/
	void generic0();
	void generic1();
	void generic2();
	void generic3();
	void generic4();
	void generic5();
	void generic6();
	void generic7();
	void generic8();
	void generic9();

	/** Generic menu
	*/
	void genericMenu();

	/** Test - go straight ahead using a defined speed.
	*/
	void goAhead();

	/** Follow a RCJ line.
	*/
	void lineFollow();

	void markers();

	/** Avoid an obstacle on line.
	*/
	void obstacleAvoid();

	/** Test for Mecanum wheels.
	*/
	void omniWheelsTest();

	/** Starts the RCJ Line run after this action selected.
	*/
	void rcjLine();

	/** Turns the robot clockwise using compass.
	@param byDegreesClockwise - turn by defined number of degrees.
	*/
	void turn(int16_t byDegreesClockwise);

	/** Follows a wall.
	*/
	void wallFollow();
};

/** Actions serve a few purposes.
- They encapsulate in classes actions robot has to perform. So, we have classes for robot's parts, but here also for non-material terms.
- No global variables are used. When an information should be shared between one (but called repeatedly) or more functions, it will be stored inside the action object. For example, all the
start conditions will be in the object itself.
- You can use inheritance to indicate relationships between actions, which indeed exist. For example, a movement can be movement straight ahead or turning.
- You can use in a consistent way actions defined for the base robot, without its code being exposed here.
- The actions are included in menus just by including a parameter in the constructor call.
- Buttons can be used to start actions, as well as menu commands. Menus are displayed both in the connected PC and a Bluetooth device, like a mobile phone, and any of the 2 can be used to
issue commands.
*/

/** Actions specific for a RobotLine robot will be defined here. They are all derived from ActionBase class (inheriting its methods and properties).
They also all feature perform() function, the one that is called to do what they are supposed to. Their constructors always call base class' constructors [ActionBase(...)] with 3 or more
parameters specified here.
First parameter is robot and is always the same.
The second one is a 3-letter shortcut that is displayed in command menu. For example "lin" will be displayed for starting the Rescue Line run. When action is not supposed to be started from menu,
it can be an empty string.
The third parameter is a name of the action, again displayed in menu. For "lin", the name is "RCJ Line", causing menu entry "line - RCJ Line" to be displayed. Again, use empty string
for no-menu actions.
The fourth pareameter is menu level. When omitted, the action will not be a part of the menu. Use 1 otherwise. Higher level numbers will display the action in submenues, not used here.
*/

/** Follow a line.
*/
class ActionLineFollow : public ActionBase {
	void perform(){ ((RobotLine*)_robot)->lineFollow(); }
public:
	ActionLineFollow(RobotLine* robot) : ActionBase(robot, "lnf", "Line follow", 1) {}
};

/** Avoid an obstacle.
*/
class ActionObstacleAvoid : public ActionBase {
	void perform() { ((RobotLine*)_robot)->obstacleAvoid(); }
public:
	bool leftOfObstacle; // Obstacle is on the robot's left side

	ActionObstacleAvoid(RobotLine* robot) : ActionBase(robot, "obs", "Obstacle avoid", 0) {}
};

/** Evacuation zone algorithm.
*/
class ActionEvacuationZone : public ActionBase {
	void perform() { ((RobotLine*)_robot)->evacuationZone(); }
public:

	ActionEvacuationZone(RobotLine* robot) : ActionBase(robot, "eva", "Evacuation zone", 1) {}
};

/** Start RCJ Line run.
*/
class ActionRCJLine : public ActionBase {
	void perform() { ((RobotLine*)_robot)->rcjLine(); }
public:
	ActionRCJLine(Robot* robot) : ActionBase(robot, "lin", "RCJ Line", 1) {}
};

/** Follow a wall.
*/
class ActionWallFollow : public ActionBase {
	void perform() { ((RobotLine*)_robot)->wallFollow(); }
public:
	ActionWallFollow(RobotLine* robot) : ActionBase(robot, "wal", "Wall follow") {}
};


// ****************** Generic actions

class ActionGeneric0 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric0(RobotLine* robot) : ActionBase(robot, "g0", "Generic 0", 8) {}
};

class ActionGeneric1 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric1(RobotLine* robot) : ActionBase(robot, "g1", "Generic 1", 8) {}
};

class ActionGeneric2 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric2(RobotLine* robot) : ActionBase(robot, "g2", "Generic 2", 8) {}
};

class ActionGeneric3 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric3(RobotLine* robot) : ActionBase(robot, "g3", "Generic 3", 8) {}
};

class ActionGeneric4 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric4(RobotLine* robot) : ActionBase(robot, "g4", "Generic 4", 8) {}
};

class ActionGeneric5 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric5(RobotLine* robot) : ActionBase(robot, "g5", "Generic 5", 8) {}
};

class ActionGeneric6 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric6(RobotLine* robot) : ActionBase(robot, "g6", "Generic 6", 8) {}
};

class ActionGeneric7 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric7(RobotLine* robot) : ActionBase(robot, "g7", "Generic 7", 8) {}
};

class ActionGeneric8 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric8(RobotLine* robot) : ActionBase(robot, "g8", "Generic 8", 8) {}
};

class ActionGeneric9 : public ActionBase {
	void perform() { ((RobotLine*)_robot)->generic0(); }
public:
	ActionGeneric9(RobotLine* robot) : ActionBase(robot, "g9", "Generic 9", 8) {}
};

/** Menu for generic actions
*/
class ActionGenericMenu : public ActionBase {
	void perform() { ((RobotLine*)_robot)->genericMenu(); }
public:
	ActionGenericMenu(Robot* robot) : ActionBase(robot, "gen", "Generic", 1) {}
};
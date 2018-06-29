#pragma once
#include "Arduino.h"

/**
Purpose: control of a motor or a motor group. The motors can use 2 PWM inputs (PWM/PWM) or a PWM and a direction input (PH/EN). 
@author MRMS team
@version 0.4 2018-06-30
Licence: You can use this code any way you like.
*/

#define MAX_MOTORS 8 // Maximum number of motors. 

#ifndef toRad
#define toRad(x) ((x) / 180.0 * PI) // Degrees to radians
#endif
#ifndef toDeg
#define toDeg(x) ((x) / PI * 180.0) // Radians to degrees
#endif

typedef bool(*BreakCondition)();

class Motors {
	bool areEnabled;
	bool isLeft[MAX_MOTORS]; // Are motors on the left side of the robot (if applicable).
	bool isReversed[MAX_MOTORS]; // Reverse rotation of motors. It is easier to change here than to swap the wires.
	bool isPWMPWM; // If true, the motor uses 2 PWM inputs. If not, a PWM and a direction input.
	int lastSpeed[MAX_MOTORS];
	uint8_t nextFree;
	uint8_t pinsPWMA[MAX_MOTORS]; // Pins for a PWM input (PWM/PWM and PH/EN)
	uint8_t pinsPWMBOrDirection[MAX_MOTORS]; // Pins for a second PWM input (PWM/PWM) or direction pins (PH/EN).
	HardwareSerial * serial; //Additional serial port
	
	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

	/** Motor control, setting a motor's speed.
	@param pwmPin - PWM pin for either PWM/PWM or PH/EN.
	@param pwmReversePinOrDirection - PWM pin (PWM/PWM) or direction pin (PH/EN).
	@param speed - Motor's speed, -100 to 100. A negative speed causes a reversed rotation.
	@param speedLimit - Speed limit, 0 to 255. For example, if set to 120, maximum speed will be 100/255=39% of the highest possible.
	*/
	void setSpeed(uint8_t pwmPin, uint8_t pwmReversePinOrDirection, int speed, uint8_t speedLimit = 255);

public:
	/**Constructor
	@param pwmPWM - If true, a PWM/PWM motor (2 digital PWM microcontroller outputs). If false, a PH/EN (phase-enable, a simple digital and a PWM digital output).
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param enabled - If set to false, motors will not rotate but will accept the commands normally. Useful for testing.
	*/
	Motors(bool pwmPWM = true, HardwareSerial *hardwareSerial = 0, bool enabled = true);

	/** Add a motor
	@param pinA - A PWM pin (either for PWM/PWM or PH/EN).
	@param pinB - Another PWM pin (PWM/PWM) or a direction pin (PH/EN). For PWM/PWM, You can also change motor's rotation by swapping pinA and pinB.
	@param isLeftMotor - The motor is on the left side of the robot, if the robot has clear left and right sides. 
	@param reversed - Changes motor's rotation direction. Either pay attention to + and - poles of the motor, or connect any and change this parameter, if the direction is not a desired one.
	*/
	void add(uint8_t pinA, uint8_t pinB, bool isLeftMotor = true, bool reversed = false);

	/** Get last motor's speed
	@param motorNumber - The motor index. Function add() assigns 0 to first motor, 1 to second, etc.
	@return speed - Sets the speed. Values -100 to 100. Negative values reverse rotation direction.
	*/
	int getSpeed(uint8_t motorNumber) { return lastSpeed[motorNumber]; }

	/** Control of a robot that has left and right wheels (and turns using speed difference of the 2 sides). The speeds of all the motors will be set.
	@param left - Sets all the left motors to the speed 'left'. Values -100 to 100. Negative values reverse rotation direction.
	@param right - The same for right motors.
	@param scaleToMaximum - If true, at least one speed ('left' or 'right') will be maximal (100 or -100) and none will be over maximal.
							For example, if the speeds are (-20, 50), the scales ones will be (-40, 100).
	@param speedLimit - Speed limit, values 0 to 100. For example, 80 will limit all the speeds to 80%. 0 will turn the motors off.
	@param verbose - Displaying data on the computer or a mobile (via Bluetooth).
	*/
	void go(double left, double right, bool scaleToMaximum = false, uint8_t speedLimit = 100, bool verbose = false);
	 
	/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels.
	@param speed - 0 to 100. 
	@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
						Values between -180 and 180. 
	@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
					numbers because a value 100 turns on all the motors at maximal speed.
	@param speedLimit - Speed limit, 0 to 100. For example, 80 will limit all the speeds to 80%. 0 will turn the motors off.
	*/
	void goOmni(float speed, float angleDegrees, float rotation, uint8_t speedLimit = 100);

	/** Setting a speed of a single motor.
	@param motorNumber - The motor index. Function add() assigns 0 to first motor, 1 to second, etc.
	@param speed - Sets the speed. Values -100 to 100. Negative values reverse rotation direction.
	@param speedLimit - Speed limit, values 0 to 100. For example, 80 will limit all the speeds to 80%. 0 will turn the motor off.
	*/
	void setSpeed(uint8_t motorNumber, int speed, uint8_t speedLimit = 100);

	/**Test
	@param type - 0 - go(30, 30), suitable for motors that use go() function.
				- 1 - go(speed, speed), speed increasing and decreasing, suitable for motors that use go() function.
				- 2 - one by one: turn on (speed 30), turn off. Suitable for all motors.
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(int type = 2, BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

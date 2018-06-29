#include "Motors.h"

/**Constructor
@param pwmPWM - If true, a PWM/PWM motor (2 digital PWM microcontroller outputs). If false, a PH/EN (phase-enable, a simple digital and a PWM digital output).
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param enabled - If set to false, motors will not rotate but will accept the commands normally. Useful for testing.
*/
Motors::Motors(bool pwmPWM, HardwareSerial *hardwareSerial, bool enabled)
{
	nextFree = 0;
	isPWMPWM = pwmPWM;
	serial = hardwareSerial;
	areEnabled = enabled;
}

/** Add a motor
@param pinA - A PWM pin (either for PWM/PWM or PH/EN).
@param pinB - Another PWM pin (PWM/PWM) or a direction pin (PH/EN). For PWM/PWM, You can also change motor's rotation by swapping pinA and pinB.
@param isLeftMotor - The motor is on the left side of the robot, if the robot has clear left and right sides.
@param reversed - Changes motor's rotation direction. Either pay attention to + and - poles of the motor, or connect any and change this parameter, if the direction is not a desired one.
*/
void Motors::add(uint8_t pinA, uint8_t pinB, bool isLeftMotorValue, bool reversed)
{
	if (nextFree >= MAX_MOTORS)
		error("Too many motors");
	pinsPWMA[nextFree] = pinA;
	pinsPWMBOrDirection[nextFree] = pinB;
	isLeft[nextFree] = isLeftMotorValue;
	lastSpeed[nextFree] = 0;
	isReversed[nextFree] = reversed;
	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	nextFree++;
}

/** Control of a robot that has left and right wheels (and turns using speed difference of the 2 sides). The speeds of all the motors will be set.
@param left - Sets all the left motors to the speed 'left'. Values -100 to 100. Negative values reverse rotation direction.
@param right - The same for right motors.
@param scaleToMaximum - If true, at least one speed ('left' or 'right') will be maximal (100 or -100) and none will be over maximal.
For example, if the speeds are (-20, 50), the scales ones will be (-40, 100).
@param speedLimit - Speed limit, values 0 to 100. For example, 80 will limit all the speeds to 80%. 0 will turn the motors off.
@param verbose - Displaying data on the computer or a mobile (via Bluetooth).
*/
void Motors::go(double left, double right, bool scaleToMaximum, uint8_t speedLimit, bool verbose) {
	//Skaliramo do 100
	double maximum = fabsf(left);
	if (fabsf(right) > maximum)
		maximum = fabsf(right);
	if ((maximum > 100.0 || scaleToMaximum) && maximum > 0.001) {
		left *= 100.0 / maximum;
		right *= 100.0 / maximum;
	}
	for (int i = 0; i < nextFree; i++)
		setSpeed(i, isLeft[i] ? left : right, speedLimit);

	if (verbose) {
#ifdef SerialPortMotors
		SerialPortMotors.print(" Sp:");
		SerialPortMotors.print(round(left));
		SerialPortMotors.print(":");
		SerialPortMotors.println(round(right));
#endif
	}
}

/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels.
@param speed - 0 to 100.
@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
Values between -180 and 180.
@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
numbers because a value 100 turns on all the motors at maximal speed.
@param speedLimit - Speed limit, 0 to 100. For example, 80 will limit all the speeds to 80%. 0 will turn the motors off.
*/
void Motors::goOmni(float speed, float angleDegrees, float rotation, uint8_t speedLimit) {
	angleDegrees -= 45;
	float angleRadians = toRad(angleDegrees);
	float si = sin(angleRadians);
	float co = cos(angleRadians);
	float xMinus135Deg = -speed * si + rotation; //135 degrees
	float x135Deg = -speed * co + rotation; //45 degrees
	float x45Deg = speed * si + rotation;  //-135 degrees
	float xMinus45Deg = speed * co + rotation;  //-45 degrees
	float speeds[4] = { x45Deg, x135Deg,xMinus135Deg,xMinus45Deg };
	
	if (speedLimit > 100)
		speedLimit = 100;
	float maxSpeed = abs(speeds[0]);
	for (int i = 1; i < 4; i++)
		if (abs(speeds[i]) > maxSpeed)
			maxSpeed = abs(speeds[i]);

	for (int i = 0; i < 4; i++)
		if (speedLimit == 0)
			setSpeed(i, 0);
		else if (maxSpeed > speedLimit )
			setSpeed(i, speeds[i] / maxSpeed * speedLimit);
		else
			setSpeed(i, speeds[i]);

}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Motors::print(String message, bool eol) {
	if (eol) {
		Serial.println(message);
		if (serial != 0)
			serial->println(message);
	}
	else {
		Serial.print(message);
		if (serial != 0)
			serial->print(message);
	}
}

/** Motor control, setting a motor's speed.
@param pwmPin - PWM pin for either PWM/PWM or PH/EN.
@param pwmReversePinOrDirection - PWM pin (PWM/PWM) or direction pin (PH/EN).
@param speed - Motor's speed, -100 to 100. A negative speed causes a reversed rotation.
@param speedLimit - Speed limit, 0 to 255. For example, if set to 120, maximum speed will be 100/255=39% of the highest possible.
*/
void Motors::setSpeed(uint8_t pwmPin, uint8_t pwmReversePinOrDirection, int speed, uint8_t speedLimit) {
	if (!areEnabled)
		return;
	speed = constrain(speed, -100, 100);
	if (speedLimit > 100)
		speedLimit = 100;
	if (speed == 0) {//Brake
		if (isPWMPWM) {
			analogWrite(pwmPin, 255);
			analogWrite(pwmReversePinOrDirection, 255);
		}
		else {
			analogWrite(pwmPin, 0);
			digitalWrite(pwmReversePinOrDirection, 0);
		}
	}
	else if (speed > 0) {
		analogWrite(pwmPin, map(abs(speed), 0, 100, 0, speedLimit / 100.0 * 255));
		if (isPWMPWM)
			analogWrite(pwmReversePinOrDirection, 0);
		else
			digitalWrite(pwmReversePinOrDirection, 0);
	}
	else {
		if (isPWMPWM) {
			analogWrite(pwmReversePinOrDirection, map(abs(-speed), 0, 100, 0, speedLimit / 100.0 * 255));
			analogWrite(pwmPin, 0);
		}
		else {
			analogWrite(pwmPin, map(abs(speed), 0, 100, 0, speedLimit / 100.0 * 255));
			digitalWrite(pwmReversePinOrDirection, 1);
		}
	}
}

/** Setting a speed of a single motor.
@param motorNumber - The motor index. Function add() assigns 0 to first motor, 1 to second, etc.
@param speed - Sets the speed. Values -100 to 100. Negative values reverse rotation direction.
@param speedLimit - Speed limit, values 0 to 100. For example, 80 will limit all the speeds to 80%. 0 will turn the motor off.
*/
void Motors::setSpeed(uint8_t motorNumber, int speed, uint8_t speedLimit) {
	if (isReversed[motorNumber])
		speed = -speed;
	lastSpeed[motorNumber] = speed;
	setSpeed(pinsPWMA[motorNumber], pinsPWMBOrDirection[motorNumber], speed, speedLimit);
}

/**Test
@param type - 0 - go(30, 30), suitable for motors that use go() function.
			- 1 - go(speed, speed), speed increasing and decreasing, suitable for motors that use go() function.
			- 2 - one by one: turn on (speed 30), turn off. Suitable for all motors.
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Motors::test(int type, BreakCondition breakWhen)
{
	switch (type) {
	case 1:
		while (breakWhen == 0 || !(*breakWhen)()) {
			for (int i = 0; i <= 100; i++) {
				go(i, i);
				if (i % 10)
					print(i, true);
				delay(100);
			}
			for (int i = 100; i >= -100; i--) {
				go(i, i);
				if (i % 10) 
					print(i, true);
				delay(100);
			}
			for (int i = -100; i < 0; i++) {
				go(i, i);
				if (i % 10)
					print(i, true);
				delay(100);
			}
		}
	case 2:
		while (breakWhen == 0 || !(*breakWhen)()) {
			for (int i = 0; i < nextFree && (breakWhen == 0 || !(*breakWhen)()); i++) {
				for (uint8_t j = 0;  j <= 1; j++){
				setSpeed(i, j == 0 ? 30 : -30);
				print("Motor " + (String)i + " 30% " + (String)(j == 0 ? "forwards" : "backwards"), true);
				delay(1000);
				setSpeed(i, 0);
				}
			}
		}
	default:
		go(30, 30);
		while (breakWhen == 0 || !(*breakWhen)())
			;
	}
	go(0, 0);
}

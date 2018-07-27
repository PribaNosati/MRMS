#pragma once
#include "Arduino.h"
#include <PWMServo.h>
#include <vector>

/**
Purpose: Using a group of servo motors.
@author MRMS team
@version 0.0 2018-26-07
Licence: You can use this code any way you like.
*/

#define MAX_SERVOS 10 //Maximum number of servo motors. 

typedef bool(*BreakCondition)();

class Servos {
	uint8_t currentPosition[MAX_SERVOS]; // Degrees.
	int nextFree;
	uint8_t parkPosition[MAX_SERVOS]; // Degrees
	HardwareSerial * serial; //Additional serial port
	PWMServo *servos[MAX_SERVOS];
	IntervalTimer servoTimer;
	std::vector<int> trajectorySteps[MAX_SERVOS];
	std::vector<uint32_t> trajectoryStepMs;

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Servos(HardwareSerial * hardwareSerial = 0);

	/** Destructor
	*/
	~Servos();

	/**Add a sensor
	@param pin - a PWM pin.
	@param parkPositionDegrees
	*/
	void add(byte pin, uint8_t = 90);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

	/** Move all servo motors into parking positions.
	*/
	void park();

	/** Remove all trajectory's steps.
	*/
	void trajectoryClear();

	/** Excecute all the steps accumulated earlier using trajectoryStep(). 
	*/
	void trajectoryRun();

	/** Add a new step along the trajectory (angles). It appends the step after all the previous. To start a fresh trajectory, call trajectoryClear().
	@param ms - duration of the step execution.
	@param ... - list of angles: for servo 0, servo 1, etc.
	*/
	void trajectoryStep(uint32_t ms...);

	/** Move servo
	@param servoNumber - Servo's ordinal number. Each call of function add() assigns an increasing number to the servos, starting with 0.
	*/
	void write(uint8_t servoNumber, uint8_t degrees);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

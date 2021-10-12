#pragma once
#include "Arduino.h"

#define PrintSpeed // If defined, a speed calculation will be displayed.

/**
Purpose: A simple proportional�integral�derivative controller (PID controller or three term controller).
@author MRMS team
@version 0.1 2018-02-19
Licence: You can use this code any way you like.
*/
class Mrm_pid
{
	float cumulativeValue = 0;
	float derivative; // A derivative component, depends on error change (not on error value).
					  // For example, if a robot follows a line and the line is the below robot's center (error 0), it will not mean that we will not have to
					  // correct the robots direction. If, at that moment, the robot is turning sharply to the left (but the current error is the mentioned 0), You will
					  // have to set the motors' speed so that the robot turns to the right. It will not turn to the right as it has a big rotational
					  // speed to the left, but the speed will be reduced. By doing nothing, robot would turn to much to the left. The proportional
					  // component would start to influence the motion but it would be too late - You would see a periodic left - right rotation.
	float integrative; // Integral component, corrects a long term error. For example when left motors are stronger causing the robot to go more to the right, when it should go straight.
	unsigned long lastCalcuationAtMicros = micros();
	float lastValue = 0;
	float proportional; // Proportional component. A bigger error causes a bigger correction.
	HardwareSerial * serial; //Additional serial port
	
	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

public:
	/** Constructor
	@param proportionalComponent - The bigger it is, the more will an error value correct the error.
	@param derivativeComponent - The bigger it is, the more will an error change correct the error.
	@param integrativeComponent - The bigger it is, the more will a cumulative error correct the error.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Mrm_pid(float proportionalComponent, float derivativeComponent, float integrativeComponent, HardwareSerial * hardwareSerial = 0);

	/** Calculation
	@param inputValue - Input value, for example an error.
	@param verbose - Display to screen.
	@param limit - Absolute return value limited to this value.
	@return - A calculated value, for example a change in robot's direction (motors' speed) needed to correct the error.
	*/
	float calculate(float inputValue, bool verbose = false, float limit = __FLT_MAX__);
};


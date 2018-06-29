#include "PID.h"

/** Constructor
@param proportionalComponent - The bigger it is, the more will an error value correct the error.
@param derivativeComponent - The bigger it is, the more will an error change correct the error.
@param integrativeComponent - The bigger it is, the more will a cumulative error correct the error.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
PID::PID(float proportionalComponent, float derivativeComponent, float integrativeComponent, HardwareSerial * hardwareSerial)
{
	proportional = proportionalComponent;
	derivative = derivativeComponent;
	integrative = integrativeComponent;
	serial = hardwareSerial;
}

/** Calculation
@param inputValue - Input value, for example an error.
@param verbose - Display to screen.
@return - A calculated value, for example a change in robot's direction (motors' speed) needed to correct the error.
*/
float PID::calculate(float valueNow, bool verbose) {
	static float cumulativeValue = 0;
	static float lastValue = 0;
	float speed = 0;
	float millisElapsed = (micros() - lastCalcuationAtMicros) / 1000.0;
	if (millisElapsed > 0.00001)
		speed = (valueNow - lastValue) / millisElapsed; // Change in 1 msec
	lastCalcuationAtMicros = micros();
	float pidCorrection = valueNow * proportional + speed * derivative + cumulativeValue * integrative; // Here the PID controller corrects the speed
	cumulativeValue += valueNow;
	if (verbose) {
#ifdef PrintSpeed
		print(" Sp="+ (String)speed + "=(" + (String)valueNow + "-" + (String)lastValue + ")/" + (String)millisElapsed);
#endif
		print(" Corr=" + (String)pidCorrection + "=" + (String)valueNow + "*" + (String)round(proportional));
		if (derivative != 0) {
			print("+" + (String)speed + "*" + (String)round(derivative));
		}
	}
	lastValue = valueNow;
	return pidCorrection;
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void PID::print(String message, bool eol) {
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

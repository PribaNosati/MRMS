#include "Servos.h"

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Servos::Servos(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

/** Destructor
*/
Servos::~Servos() {
}

/**Add a sensor
@param pin - a PWM pin.
@param parkPositionDegrees
*/
void Servos::add(byte pin, uint8_t parkPositionDegrees)
{
	if (nextFree >= MAX_SERVOS)
		error("Too many servos");
	servos[nextFree] = new PWMServo();
	servos[nextFree]->attach(pin);
	currentPosition[nextFree] = parkPositionDegrees;
	parkPosition[nextFree] = parkPositionDegrees;
	nextFree++;

}

/** Move all servo motors into parking positions.
*/
void Servos::park() {
	trajectoryClear();
	for (uint8_t i = 0; i < nextFree; i++)
		trajectorySteps[i].push_back(parkPosition[i]);
	trajectoryStepMs.push_back(1000);
	trajectoryRun();
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Servos::print(String message, bool eol) {
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

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Servos::test(BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {

		}
	}
}

/** Remove all trajectory's steps.
*/
void Servos::trajectoryClear() {
	for (uint8_t i = 0; i < nextFree; i++) 
		trajectorySteps[i].clear();
}

/** Excecute all the steps accumulated earlier using trajectoryStep().
*/
void Servos::trajectoryRun() {
	const bool VERBOSE = false;
	for (uint16_t i = 0; i < trajectorySteps[0].size(); i++) {
		
		if (VERBOSE) {
			for (uint8_t j = 0; j < nextFree; j++) {
				uint8_t deg = trajectorySteps[j][i];
				print((String)(int)deg + " ");
			}
		}

		uint32_t startMs = millis();
		uint32_t durationMs = trajectoryStepMs[i];
		while (millis() < startMs + durationMs) {
			uint32_t ms = millis() - startMs;
			for (uint8_t j = 0; j < nextFree; j++) {
				uint8_t deg = trajectorySteps[j][i];
				write(j, currentPosition[j] * (durationMs - ms) / (float)durationMs + deg * ms / (float)durationMs);
			}
		}
		if (VERBOSE)
			print("", true);

		for (uint8_t j = 0; j < nextFree; j++) 
			currentPosition[j] = trajectorySteps[j][i];
	}
}

/** Add a new step along the trajectory (angles). It appends the step after all the previous. To start a fresh trajectory, call trajectoryClear().
@param ms - duration of the step execution.
@param ... - list of angles: for servo 0, servo 1, etc.
*/
void Servos::trajectoryStep(uint32_t ms...) {
	va_list args;
	va_start(args, ms);
	for (uint8_t i = 0; i < nextFree; i++){
		uint8_t deg = (uint8_t)va_arg(args, int);
		trajectorySteps[i].push_back(deg);
	}
	trajectoryStepMs.push_back(ms);

	va_end(args);
}

/** Move servo
@param servoNumber - Servos's ordinal number. Each call of function add() assigns an increasing number to the servos, starting with 0.
*/
void Servos::write(uint8_t servoNumber, uint8_t degrees) {
	if (nextFree >= MAX_SERVOS)
		error("Servo's index wrong");
	servos[servoNumber]->write(degrees);
}

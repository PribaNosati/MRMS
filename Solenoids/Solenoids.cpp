#include "Solenoids.h"

/** Add a solenoid
@param pin - a digital pins the solenoid uses
*/
void Solenoids::add(uint8_t pin)
{
	if (nextFree >= MAX_SOLENOIDS)
		error("Too many solenoids");
	pins[nextFree] = pin;
	pinMode(pin, OUTPUT);
	nextFree++;
}

/** Turn off
@param solenoidNumber - Solenoid's index. Function add() assigns 0 to first solenoid, 1 to second, etc.
*/
void Solenoids::off(uint8_t solenoidNumber) {
	if (solenoidNumber >= nextFree)
		error("Solenoid's index too big.");
	digitalWrite(pins[solenoidNumber], LOW);
}

/** Turn on
@param solenoidNumber - Solenoid's index. Function add() assigns 0 to first solenoid, 1 to second, etc.
*/
void Solenoids::on(uint8_t solenoidNumber) {
	if (solenoidNumber >= nextFree)
		error("Solenoid's index too big.");
	digitalWrite(pins[solenoidNumber], HIGH);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Solenoids::test(BreakCondition breakWhen)
{
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			on(i);
			delay(20);
			off(i);
			delay(1000);
		}
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Solenoids::Solenoids(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Solenoids::~Solenoids() {}

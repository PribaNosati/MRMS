#include "Switches.h"

/** Add a switch
@param pin - a digital pins the switch uses
*/
void Switches::add(uint8_t pin)
{
	if (nextFree >= MAX_SWITCHES)
		error("Too many switches");
	pins[nextFree] = pin;
	pinMode(pin, INPUT);
	nextFree++;
}

/** Pin
@param sensorNumber - Sensor's index. Function add() assigns 0 to first switch, 1 to second, etc.
@return - Pin number
*/
uint8_t Switches::pin(uint8_t sensorNumber) {
	return pins[sensorNumber];
}

/** Is on
@param sensorNumber - Sensor's index. Function add() assigns 0 to first switch, 1 to second, etc.
@return - true if on.
*/
bool Switches::on(uint8_t sensorNumber) {
	if (sensorNumber >= nextFree)
		error("Switch's index too big.");
	return digitalRead(pins[sensorNumber]);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Switches::test(BreakCondition breakWhen)
{
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			Serial.print(on(i));
			if (serial != 0) serial->print(on(i));
		}
		Serial.println();
		if (serial != 0) serial->println();
		delay(100);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Switches::Switches(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Switches::~Switches() {}

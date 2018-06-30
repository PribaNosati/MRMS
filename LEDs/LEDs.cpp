#include "LEDs.h"

/**Add a LED
@param pin - Digital pin the LED uses
*/
void LEDs::add(uint8_t pin)
{
	if (nextFree >= MAX_LEDS)
		error("Too many LEDs");
	pins[nextFree] = pin;
	isOn[nextFree] = false;
	pinMode(pin, OUTPUT);
	nextFree++;
}

/**Turn off a LED
@param ledNumber - LED index. Function add() assigns 0 to first LED, 1 to second, etc.
*/
void LEDs::off(uint8_t ledNumber) {
	if (ledNumber == 0xFF) 
		for (uint8_t i = 0; i < nextFree; i++)
			off(i);
	else {
		isOn[ledNumber] = false;
		digitalWrite(pins[ledNumber], LOW);
	}
}

/**Turn on a LED
@param ledNumber - LED index. Function add() assigns 0 to first LED, 1 to second, etc.
@param intensity - 0 - 255
*/
void LEDs::on(uint8_t ledNumber, uint8_t intensity) {
	if (ledNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			on(i);
	else {
		isOn[ledNumber] = true;
		if (intensity == 255)
			digitalWrite(pins[ledNumber], HIGH);
		else
			analogWrite(pins[ledNumber], intensity);
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void LEDs::test(BreakCondition breakWhen)
{
	uint32_t lastChangeMs = 0;
	off();
	while (breakWhen == 0 || !(*breakWhen)()) {
		if (millis() - lastChangeMs > 300) {
			for (int i = 0; i < nextFree; i++) {
				toggle(i);
				Serial.print(isOn[i] ? 0 : 1);
				if (serial != 0) serial->print(isOn[i] ? 0 : 1);
			}
			Serial.println();
			if (serial != 0) serial->println();
			lastChangeMs = millis();
		}
	}
}

/**Toggle u LED
@param ledNumber - LED index. Function add() assigns 0 to first LED, 1 to second, etc.
*/
void LEDs::toggle(uint8_t ledNumber) {
	if (isOn[ledNumber])
		off(ledNumber);
	else
		on(ledNumber);
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
LEDs::LEDs(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

LEDs::~LEDs()
{
}

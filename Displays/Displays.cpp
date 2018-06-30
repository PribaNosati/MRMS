#include "Displays.h"

/**Add a display
@param address - I2C address
*/
void Displays::add(uint8_t address)
{
	if (nextFree >= MAX_DISPLAYS)
		error("Too many displays");
	matrix[nextFree] = new Adafruit_BicolorMatrix(true);
	uint32_t ms = millis();
	matrix[nextFree]->begin(address);  // pass in the address
	if (millis() - ms > 100)
		timeout();
	nextFree++;
}

/**Clears display
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
*/
void Displays::clear(uint8_t displayNumber) {
	uint32_t ms = millis();
	matrix[displayNumber]->clear();
	if (millis() - ms > 100)
		timeout();
}

/** Displays a character
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param character - character to be displayed.
@param color - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white.
*/
void Displays::displayChar(uint8_t displayNumber, String character, uint16_t color) {
	uint32_t ms = millis();
	matrix[displayNumber]->setTextSize(1);
	matrix[displayNumber]->setTextColor(color);
	matrix[displayNumber]->clear();
	matrix[displayNumber]->setCursor(0, 0);
	matrix[displayNumber]->print(character);
	matrix[displayNumber]->writeDisplay();
	if (millis() - ms > 100)
		timeout();
}

/**Draws a bitmap
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param x - x coordinate of the top left corner.
@param y - y coordinate of the top left corner.
@param bitmap - pointer to a stored bitmap.
@param w - width, number of pixels.
@param h - height, number of pixels.
@param color - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white.
*/
void Displays::drawBitmap(uint8_t displayNumber, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
	uint32_t ms = millis();
	matrix[displayNumber]->drawBitmap(x, y, bitmap, w, h, color);
	if (millis() - ms > 100)
		timeout();
}

/**Fills screen
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param color - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white
*/
void Displays::fillScreen(uint8_t displayNumber, uint16_t color) {
	uint32_t ms = millis();
	matrix[displayNumber]->fillScreen(color);
	if (millis() - ms > 100)
		timeout();
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Displays::printUART(String message, bool eol) {
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

/** Prints character like Arduino print().
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param ch - character.
@param type
*/
void Displays::print(uint8_t displayNumber, char ch, int type) {
	uint32_t ms = millis();
	matrix[displayNumber]->print(ch, type);
	if (millis() - ms > 100)
		timeout();
}

/** Prints a string.
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param str - string to be displayed.
*/
void Displays::print(uint8_t displayNumber, String str) {
	uint32_t ms = millis();
	matrix[displayNumber]->print(str);
	if (millis() - ms > 100)
		timeout();
}

/** Scrolls a string
@param displayNumber - Display number.
@param message - string to be scrolled.
@param lastXScroll - total number of pixel columns to be scrolled. A letter has a certain number of columns, like 5.
@param fontSize - velièina fonta, implicitno 1
@param delayMs - ms izmeðu pomaka, implicitno 70
*/
void Displays::scrollString(uint8_t displayNumber, String message, int lastXScroll, uint8_t fontSize, uint16_t delayMs) {
	matrix[displayNumber]->setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
	matrix[displayNumber]->setTextSize(fontSize);
	matrix[displayNumber]->setTextColor(LED_GREEN);
	for (int8_t x = 7; x >= -lastXScroll; x--) {
		matrix[displayNumber]->clear();
		matrix[displayNumber]->setCursor(x, 0);
		matrix[displayNumber]->print(message);
		matrix[displayNumber]->writeDisplay();
		delay(delayMs);
	}
}

/** Sets cursor's position
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param x - x coordinate
@param y - y coordinate
*/
void Displays::setCursor(uint8_t displayNumber, int16_t x, int16_t y) {
	uint32_t ms = millis();
	matrix[displayNumber]->setCursor(x, y);
	if (millis() - ms > 100)
		timeout();
}

/** Sets text color
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param c - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white
*/
void Displays::setTextColor(uint8_t displayNumber, uint16_t c) {
	uint32_t ms = millis();
	matrix[displayNumber]->setTextColor(c);
	if (millis() - ms > 100)
		timeout();
}

/** Sets text size
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param size - multiplies text size by this number.
*/
void Displays::setTextSize(uint8_t displayNumber, uint8_t s) {
	uint32_t ms = millis();
	matrix[displayNumber]->setTextSize(s);
	if (millis() - ms > 100)
		timeout();
}

/** Sets text wrap
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
@param w - wrap or not.
*/
void Displays::setTextWrap(uint8_t displayNumber, boolean w) {
	uint32_t ms = millis();
	matrix[displayNumber]->setTextWrap(w);
	if (millis() - ms > 100)
		timeout();
}

/**Test
@param displayNumber
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Displays::test(uint8_t sensorNumber, BreakCondition breakWhen)
{
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			test(i);
	else {
		while (breakWhen == 0 || !(*breakWhen)()) {
			setTextWrap(sensorNumber, false);  // we dont want text to wrap so it scrolls nicely
			setTextSize(sensorNumber, 1);
			setTextColor(sensorNumber, LED_GREEN);
			for (int8_t x = 7; x >= -36; x--) {
				clear(sensorNumber);
				setCursor(sensorNumber, x, 0);
				print(sensorNumber, "Hello");
				writeDisplay(sensorNumber);
				delay(100);
			}
		}
	}
}

/** Handles timeout
*/
void Displays::timeout() {
	printUART("I2C timeout!", true);
}

/**Write to display
@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
addes display.
*/
void Displays::writeDisplay(uint8_t displayNumber) {
	uint32_t ms = millis();
	matrix[displayNumber]->writeDisplay();
	if (millis() - ms > 100)
		timeout();
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Displays::Displays(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Displays::~Displays()
{
}

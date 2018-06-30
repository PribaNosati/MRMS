#pragma once
#include "Arduino.h"
#include "Adafruit_LEDBackpack.h" //Included original Adafruit library.

/**
Purpose: Adafruit tricolor display (Adafruit's product id 902) control. In fact, a wrapper for Adafruit library.
@author MRMS team
@version 0.1 2018-05-15
Licence: You can use this code any way you like.
*/

#define MAX_DISPLAYS 1 // Maximum number of displays. 
#define I2C_TIMEOUT 100 // Maximum allowed ms to wait for a I2C command to complete

typedef bool(*BreakCondition)();

class Displays
{
	Adafruit_BicolorMatrix *matrix[MAX_DISPLAYS]; //Pointers to the devices
	int nextFree; //Number of displays - 1
	HardwareSerial * serial; //Additional serial port

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void printUART(String message, bool eol = false);

	/** Handles timeout
	*/
	void timeout();

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	Displays(HardwareSerial * hardwareSerial = 0);

	/**Destructor
	*/
	~Displays();

	/**Add a display
	@param address - I2C address
	*/
	void add(uint8_t address);

	/**Clears display
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	*/
	void clear(uint8_t displayNumber);

	/** Displays a character
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param character - character to be displayed.
	@param color - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white
	*/
	void displayChar(uint8_t displayNmuber, String character, uint16_t color = LED_GREEN);

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
	void drawBitmap(uint8_t displayNumber, int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);

	/**Fills screen
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param color - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white
	*/
	void fillScreen(uint8_t displayNumber, uint16_t color);

	/** Prints character like Arduino print().
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param ch - character.
	@param type
	*/
	void print(uint8_t displayNumber, char ch, int type = BYTE);

	/** Prints a string.
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param str - string to be displayed.
	*/
	void print(uint8_t displayNumber, String str);

	/** Scrolls a string
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
	addes display.
	@param message - string to be scrolled.
	@param lastXScroll - total number of pixel columns to be scrolled. A letter has a certain number of columns, like 5.
	@param fontSize - veličina fonta, implicitno 1
	@param delay - ms između pomaka, implicitno 60
	*/
	void scrollString(uint8_t displayNumber, String message, int lastXScroll, uint8_t fontSize = 1, uint16_t delayMs = 60);

	/** Sets cursor's position
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param x - x coordinate
	@param y - y coordinate
	*/
	void setCursor(uint8_t displayNumber, int16_t x, int16_t y);

	/** Sets text color
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param c - for example 0 is black, 0x1F blue, 0xF800 red, 0x07E0 green, 0x07FF cyan, 0xF81F magenta, 0xFFE0 yellow, 0xFFFF white
	*/
	void setTextColor(uint8_t displayNumber, uint16_t c);

	/** Sets text size
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param size - multiplies text size by this number.
	*/
	void setTextSize(uint8_t displayNumber, uint8_t size);

	/** Sets text wrap
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param w - wrap or not.
	*/
	void setTextWrap(uint8_t displayNumber, boolean w);

	/**Test
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted. 0xFF - test all displays.
	*/
	void test(uint8_t sensorNumber = 0xFF, BreakCondition breakWhen = 0);

	/**Write to display
	@param displayNumber - display's ordinal number. Each call of function add() assigns an increasing number to the
		addes display.
	*/
	void writeDisplay(uint8_t displayNumber);
};

//Declaration of error function. Definition is in Your code.
void error(String message);



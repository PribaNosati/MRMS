#pragma once

//#include <Arduino.h>
//#include <strarg.h>

// A command can be executed (processed) after user enters its shortcut, but also act as a state machine.
struct Command {
	bool firstProcess = true;
	char shortcut[4];
	char text[20];
	void (*pointer)();
	uint8_t menuLevel;
};

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void print(const char* fmt, ...);

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp);
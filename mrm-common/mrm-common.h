#pragma once

//#include <Arduino.h>

// A command can be executed (processed) after user enters its shortcut, but also act as a state machine.
struct Command {
	bool firstProcess = true;
	char shortcut[4];
	char text[20];
	void (*pointer)();
	uint8_t menuLevel;
};

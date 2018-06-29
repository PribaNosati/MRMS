#include "Commands.h"
#include <string.h>

/**Add a sensor
@param shortcut - Command's shortcut.
@param command - C function that this command executes. It can be 0, in which case nothing is executed and the normal program flow will continue.
@param pin - Digital pin that a switch uses for starting this command.
@param menuId - Menu id. Top level is 0.
*/
void Commands::add(const char * shortcut, Command command, const char * name, uint8_t pin, uint8_t menuId)
{
	if (nextFree >= MAX_COMMANDS)
		error("Too many commands");
	if (strlen(shortcut) > MAX_SHORTCUT_CHARACTERS)
		error("Command shortcut too long");
	if (strlen(name) > MAX_NAME_CHARACTERS)
		error("Command name too long");
	stpcpy(shortcuts[nextFree], shortcut);
	strcpy(names[nextFree], name);
	pins[nextFree] = pin;
	menuIds[nextFree] = menuId;
	commands[nextFree] = command;
	if (pin != 255)
		pinMode(pin, INPUT);
	nextFree++;

	if (locked && password == "")
		locked = false;
}

/**
Reports that a command is available or it is being typed.
@param resetIfChar - for that character, clear inputs (and it will be no more available).
@return - true if typing in progress or complete, or the switch pressed.
*/
bool Commands::available(uint8_t resetIfChar)
{
	if (Serial.available() || (serial != 0 && serial->available())) {
		if ((Serial.available() && Serial.peek() == resetIfChar) ||
			(serial != 0 && serial->available() && serial->peek() == resetIfChar)) {
			while (Serial.available())
				Serial.read();
			while (serial != 0 && serial->available())
				serial->read();
		}
		return true;
	}

	const bool switchBreaks = true; // Switch will not be reliable if the motors cause a big voltage drop.
	return switchBreaks && button() != -1;
}

/** Switch pressed?
@param verbose - Displays data.
@return - Command's index. -1 if no switch pressed.
*/
int16_t Commands::button(bool verbose) {
	static unsigned long pressedAtMs = 0;
	int16_t commandIndex = -1;

	if (millis() - pressedAtMs > 300) {// Debounce.
		for (int i = 0; i < nextFree && commandIndex == -1; i++)
			if (pins[i] != 255 && digitalRead(pins[i]) == HIGH) {
				commandIndex = i;
				pressedAtMs = millis();
				if (verbose) 
					print("Button " + (String)i + " pressed.");
				break;
			}
	}
	return commandIndex;
}

/**
Lists all the commands. Convenient for menu display when the program is stopped, using function prompt(), so that user can see all the commands.
*/
void Commands::list()
{
	if (locked) {
		print("\n\rPassword:", true);
		return;
	}
	const uint8_t COLUMN_COUNT = 3;
	char formatString[10];
	sprintf(formatString, "%%%i-s %%%i-s", MAX_SHORTCUT_CHARACTERS, MAX_NAME_CHARACTERS);
	print("", true);
	uint8_t col = 0;
	for (int i = 0; i < nextFree; i++)
		if (menuIds[i] == currentMenuId)
		{
			char buffer[MAX_NAME_CHARACTERS + MAX_SHORTCUT_CHARACTERS + 2];
			sprintf(buffer, formatString, shortcuts[i], names[i]);
			print(buffer);

			col++;
			if (col == COLUMN_COUNT) {
				print("", true);
				col = 0;
			}
		}
	if (col != 0)
		print("", true);
}


/** Print to all serial ports
@param message
@param eol - end of line
*/
void Commands::print(String message, bool eol) {
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

/**
Stops the program (or stops conditionally), waits for a command's shortcut, and executes the command.
@param breakProgram - If true, stops unconditionally. If false, stops only if command is being typed.
@param commandWhenBreak - A command that has to be executed when the program is interrupted, like stopping the motors.
@return - The command was executed.
*/
bool Commands::prompt(bool breakProgram, Command commandWhenBreak)
{
	if (!breakProgram) {
		if (available()) //Only when typing or the switch pressed.
			print(" Break.");
		else
			return false;
	}

	if (commandWhenBreak != 0)
		(*commandWhenBreak)();

	char buffer[MAX_SHORTCUT_CHARACTERS + 1];
	for (int i = 0; i <= MAX_SHORTCUT_CHARACTERS; i++)
		buffer[i] = '\0';
	uint8_t position = 0;
	int commandIndex = -1;
	uint32_t lastUserActionMs = 0;
	while (commandIndex == -1) {

		char ch = '\0';
		if (Serial.available()) {
			ch = Serial.read();
			lastUserActionMs = millis();
		}
		else if (serial != 0 && serial->available()) {
			ch = serial->read();
			lastUserActionMs = millis();
		}

		if (ch != '\0') {
			buffer[position] = ch;
			print(buffer[position]);

			//Locked?
			if (locked) {
				if ((String)buffer == password){
					print(" Unlocked.", true);
					locked = false;
					position = 0;
					continue;
				}
			}
			else{

			//Compare to all the commands.
				for (int i = 0; i < nextFree; i++) {
					if (!strcmp(buffer, shortcuts[i])) {
						commandIndex = i;
						break;
					}
				}
			}

			if (commandIndex == -1 && ++position == MAX_SHORTCUT_CHARACTERS) {
				print(locked ? " Locked" : "Command too long or nonexistent - reset.", true);
				position = 0;
				if (locked && passwordWrongTries++ >= 2) {
					print("20 sec. delay.");
					delay(20000);
					print(" Over.", true);
				}
			}
		}

		if (commandIndex == -1)
			commandIndex = button(true);

		if (position != 0 && millis() - lastUserActionMs > TIMEOUT_MS) {
			print(" Timeout.", true);
			position = 0;
		}
	}

	print(" Command found: ");
	print(shortcuts[commandIndex], true);
	currentCommand = commandIndex;
	if (commands[commandIndex] != 0)
		return (*(commands[commandIndex]))();
	else
		return false;
}

/** Menu id for the list() function
@param menuId - Menu id. Top level is 0.
*/
void Commands::menuIdSet(uint8_t menuId) {
	currentMenuId = menuId;
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param passwordNeeded - if empty string, no password. Otherwise password required before the first command.
*/
Commands::Commands(HardwareSerial * hardwareSerial, String passwordNeeded){
	serial = hardwareSerial;
	nextFree = 0;
	password = passwordNeeded;
}

Commands::~Commands(){}

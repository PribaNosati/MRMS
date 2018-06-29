#pragma once
#include "Arduino.h"

/**
Purpose: controlling a robot using text commands or switches, using a mobile or a computer, and a Bluetooth terminal.
@author MRMS team
@version 0.6 2018-06-18
Licence: You can use this code any way you like.
*/

#define MAX_COMMANDS 100 //Maximum number of commands.
#define MAX_NAME_CHARACTERS 20 //Maximum number of characters for a command's name.
#define MAX_SHORTCUT_CHARACTERS 3 //Maximum number of characters for a command's shortcut.
#define TIMEOUT_MS 2000//If input paused (2000 ms here), reset it.

typedef bool (*Command)();

class Commands
{
	Command commands[MAX_COMMANDS]; //C functions that commands use.
	uint8_t currentCommand;
	uint8_t menuIds[MAX_COMMANDS];
	uint8_t currentMenuId = 0;
	bool locked = true; //Before (optional) password entered.
	char names[MAX_COMMANDS][MAX_NAME_CHARACTERS + 1]; //Commands names
	int nextFree;
	String password;
	uint8_t passwordWrongTries = 0;
	uint8_t pins[MAX_COMMANDS]; //Pins that switches use for execution of commands.
	HardwareSerial *serial; //Additional serial port
	char shortcuts[MAX_COMMANDS][MAX_SHORTCUT_CHARACTERS + 1]; //Typing these shorcuts, You execute the commands.

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);
	
public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication. Example:
		Commands(&Serial2);
	@param passwordNeeded - if empty string, no password. Otherwise password required before the first command.
	*/
	Commands(HardwareSerial * hardwareSerial = 0, String passwordNeeded = "");

	/**Destructor
	*/
	~Commands();

	/**Add a command
	@param shortcut - Command's shortcut.
	@param command - C function that this command executes. It can be 0, in which case nothing is executed and the normal program flow will continue.
	@param pin - Digital pin that a switch uses for starting this command.
	@param menuId - Menu id. Top level is 0.
	*/
	void add(const char * shortcut, Command command, const char * name = 0, uint8_t pin = 255, uint8_t menuId = 0);

	/**
	Reports that a command is available or it is being typed.
	@param resetIfChar - for that character, clear inputs (and it will be no more available).
	@return - true if typing in progress or complete, or the switch pressed.
	*/
	bool available(uint8_t resetIfChar = 0);

	/** Switch pressed?
	@param verbose - Displays data.
	@return - Command's index. -1 if no switch pressed.
	*/
	int16_t button(bool verbose = false);

	/** 
	Lists all the commands. Convenient for menu display when the program is stopped, using function prompt(), so that user can see all the commands.
	*/
	void list();

	/** Menu id for the list() function
	@param menuId - Menu id. Top level is 0.
	*/
	void menuIdSet(uint8_t menuId = 0);

	/**
	Stops the program (or stops conditionally), waits for a command's shortcut, and executes the command.
	@param breakProgram - If true, stops unconditionally. If false, stops only if command is being typed.
	@param commandWhenBreak - A command that has to be executed when the program is interrupted, like stopping the motors.
	@return - The command was executed.
	*/
	bool prompt(bool breakProgram = true, Command commandWhenBreak = 0);
};

//Declaration of error function. Definition is in Your code.
void error(String message);

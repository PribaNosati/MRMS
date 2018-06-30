#pragma once
#include "Arduino.h"

/**
Purpose: time measurements
@author MRMS team
@version 0.0 2018-03-20
Licence: You can use this code any way you like.
*/

class TimeMeasure {
private:
	uint16_t cnt = 0;
	uint32_t reportLastMs = 0;
	HardwareSerial * serial; //Additional serial port
	uint32_t startMs = millis();
	
	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	TimeMeasure(HardwareSerial * serial = 0);
	
	/**Destructor*/
	~TimeMeasure();
	
	/**Displays progress
	@param intervalMs - displays progress after this time elapses.
	@param displayTime - displays seconds
	*/
	void heartBeat(uint16_t intervalMs = 500, bool displayTime = false);
	
	/**Resets counters
	*/
	void reset();
	
	/**End of run report
	@param printIfMsElapsed - conditional report, only when this interval elapsed.
	@param reset - reset after report
	*/
	void report(uint32_t printIfMsElapsed = 0, bool andReset = true);
	
	/**This function must be invoked periodically, in each pass of the code segment being measured.
	*/
	void step();
};

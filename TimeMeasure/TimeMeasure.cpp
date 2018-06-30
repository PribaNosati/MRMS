#include "TimeMeasure.h"

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
TimeMeasure::TimeMeasure(HardwareSerial * hardwareSerial){
	serial = hardwareSerial;
	reset();
}

/**Destructor*/
TimeMeasure::~TimeMeasure() {}

/**Displays progress
@param intervalMs - displays progress after this time elapses.
@param displayTime - displays seconds
*/
void TimeMeasure::heartBeat(uint16_t intervalMs, bool displayTime) {
	static uint32_t heartBeatLastMs = 0;
	static uint8_t cnt = 0;
	if (millis() - heartBeatLastMs > intervalMs) {
		if (displayTime && cnt == 0) 
			print((String)(millis() / 1000) + " s");
		print(".");
		if (cnt++ > 40) {
			print("", true);
			cnt = 0;
		}
		heartBeatLastMs = millis();
	}
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void TimeMeasure::print(String message, bool eol) {
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

/**Resets counters
*/
void TimeMeasure::reset() {
	cnt = 0;
	startMs = millis();
}

/**This function must be invoked periodically, in each pass of the code segment being measured.
*/
void TimeMeasure::step() {
	if (cnt == 0)
		startMs = millis();
	cnt++;
}

/**End of run report
@param printIfMsElapsed - conditional report, only when this interval elapsed.
@param reset - reset after report
*/
void TimeMeasure::report(uint32_t printIfMsElapsed, bool andReset) {
	if (printIfMsElapsed == 0 || millis() - reportLastMs > printIfMsElapsed) {
		print("Number of cycles: " + (String)cnt + ", ");
		uint16_t fps = round(cnt / ((millis() - startMs) / 1000.0));
		print((String)fps + " FPS.");
		reportLastMs = millis();
		if (andReset)
			reset();
	}
}

#include <mrm-common.h>

// Global variables to reduce coupling among classes and reduce number of parameters
BluetoothSerial *serialBT = NULL;
char errorMessage[60] = ""; // Global variable enables functions to set it although not passed as parameter

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}


/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp) {
	if (strlen(fmt) >= 100)
		return;
	static char buffer[100];
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serialBT != NULL)
		serialBT->print(buffer);
}
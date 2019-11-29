//#include <mrm-common.h>

///** Print to all serial ports
//@param fmt - C format string
//@param ... - variable arguments
//*/
//void print(const char* fmt, ...) {
//	va_list argp;
//	va_start(argp, fmt);
//	vprint(fmt, argp);
//	va_end(argp);
//}
//
///** Print to all serial ports, pointer to list
//*/
//void vprint(const char* fmt, va_list argp) {
//
//	static char buffer[100]; // Caution !!! No checking if longer than 100!
//	vsprintf(buffer, fmt, argp);
//
//	Serial.print(buffer);
//	if (serial != 0)
//		serial->print(buffer);
//}

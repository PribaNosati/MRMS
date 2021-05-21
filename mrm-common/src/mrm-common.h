#pragma once
#define RADIO 1 // 0 - no radio, 1 Bluetooth, 2 WiFi
#include <Arduino.h>
#if RADIO == 1
#include <BluetoothSerial.h>
#endif

extern char errorMessage[60];

/** Print to all serial ports
@param fmt - C format string: 
	%c - character,
	%i - integer,
	%s - string.
@param ... - variable arguments
*/
void print(const char* fmt, ...);

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp);
#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>

extern char errorMessage[60];

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void print(const char* fmt, ...);

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp);
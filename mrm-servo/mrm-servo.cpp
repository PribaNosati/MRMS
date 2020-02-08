#include "mrm-servo.h"

extern char errorMessage[];

/** Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_servo::Mrm_servo(BluetoothSerial* hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_servo::~Mrm_servo()
{
}

/** Add a servo motor
@param gpioPin - pin number, allowed: 0 - 19, 21 - 23, 25 - 27, 32 - 36, 39
@param deviceName - device's name
@param timerWidth - timer width in bits, 1 - 16
*/
void Mrm_servo::add(uint8_t gpioPin, char* deviceName, uint8_t timerWidth)
{
	if (nextFree >= MAX_SERVO_COUNT) {
		strcpy(errorMessage, "Too many servo motors");
		return;
	}

	if (deviceName != 0) {
		if (strlen(deviceName) > 9) {
			strcpy(errorMessage, "Device name too long");
			return;
		}
		strcpy(nameThis[nextFree], deviceName);
	}

	timerWidthBits = timerWidth;

	// Servo: 20 ms period, duty 1 - 2 ms. 1.5 ms - neutral position.
#define FREQUENCY_HZ 50
	double resFreq = ledcSetup(nextFree, FREQUENCY_HZ, timerWidthBits); // nextFree is channel number, which can be 0 - 15.
	ledcAttachPin(gpioPin, nextFree); // GPIO 16 assigned to channel 1

	nextFree++;
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Mrm_servo::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_servo::test(BreakCondition breakWhen)
{
	const uint8_t ms = 12;
	while (breakWhen !=0 && !breakWhen()) {

		for (int16_t i = 0; i <= 180; i += 5) {
			servoWrite(i);
			print("%i\n\r", i);
			delay(ms);
		}

		for (int16_t i = 180; i >= 0; i -= 5) {
			servoWrite(i);
			print("%i\n\r", i);
			delay(ms);
		}
	}

	print("\n\rTest over.\n\r");
}

/** Print to all serial ports, pointer to list
*/
void Mrm_servo::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}

/** Move servo
@param degrees - Servo's target angle, 0 - 180
@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
*/
void Mrm_servo::servoWrite( uint16_t degrees, uint8_t servoNumber) {
	if (servoNumber >= nextFree) {
		strcpy(errorMessage, "Servo doesn't exist");
		return;
	}
	degrees = constrain(degrees, 0, 180);
	uint16_t period = (1 << timerWidthBits) - 1;
	ledcWrite(servoNumber, map(degrees, 0, 180, period * 0.025, period * 0.125));// /20 /10
	//ledcWrite(1, map(degrees, 0, 180, PERIOD / 20, PERIOD / 10));
}

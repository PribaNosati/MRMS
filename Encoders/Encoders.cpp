#include "Encoders.h"

volatile uint32_t encoderCounters[MAX_ENCODERS];//Counters

//Interrupt functions
void f0() { encoderCounters[0]++; }
void f1() { encoderCounters[1]++; }
void f2() { encoderCounters[2]++; }
void f3() { encoderCounters[3]++; }
void f4() { encoderCounters[4]++; }
void f5() { encoderCounters[5]++; }
void f6() { encoderCounters[6]++; }
void f7() { encoderCounters[7]++; }
ArgumenlessFunction encoderHandlers[] = { f0, f1, f2, f3, f4, f5, f6, f7 };

/**Add an encoder
@param pin - Pin the encoder uses.
*/
void Encoders::add(uint8_t pin)
{
	if (nextFree >= MAX_ENCODERS)
		error("Too many encoders");

	if (nextFree >= 8)
		error("Not enough interrupt handlers.");

	pins[nextFree] = pin;
	encoderCounters[nextFree] = 0;
	pinMode(pin, INPUT);
	attachInterrupt(pin, encoderHandlers[nextFree], RISING);
	nextFree++;
}

/**Backup positions
*/
void Encoders::backup() {
	for (uint8_t i = 0; i < MAX_ENCODERS; i++)
		backupSteps[i] = encoderCounters[i];
}

/**Read a counter.
@param encoderNumber - Encoder's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
uint32_t Encoders::counter(int encoderNumber)
{
	if (encoderNumber >= nextFree)
		error("Out of range");
	return encoderCounters[encoderNumber];
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Encoders::print(String message, bool eol) {
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

/**Resets all the counters.
@param to0 - reset to 0. Otherwise to backup. In that case, backup() function had to be called before.
*/
void Encoders::reset(bool to0) {
	for (uint8_t i = 0; i < nextFree; i++)
		encoderCounters[i] = to0 ? 0 : backupSteps[i];
}

/**Set a counter.
@param encoderNumber - Encoder's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param value - Value
*/
void Encoders::set(int encoderNumber, uint32_t value) {
	encoderCounters[encoderNumber] = value;
}

/**Test
@param motorStartFunction - Motor starting function. It is necessary because encoders produce no results without the motors revolving.
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Encoders::test(ArgumenlessFunction motorStartFunction, BreakCondition breakWhen)
{
	reset();
	(*motorStartFunction)();
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			if (i != 0) 
				print(" ");
			print(encoderCounters[i]);
		}
		print("", true);
		delay(100);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Encoders::Encoders(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Encoders::~Encoders(){}

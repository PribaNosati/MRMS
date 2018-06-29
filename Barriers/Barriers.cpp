#include "Barriers.h"
#include <EEPROM.h>

/**Add a barrier
@param pin - Analog pin. Barrier is connected here.
@param lowestValue - The lowest value the sensor produces.
@param highestValue - The highest.
*/
void Barriers::add(uint8_t pin, uint16_t lowestValue, uint16_t highestValue)
{
	if (nextFree >= MAX_BARRIERS)
		error("Too many barriers");
	pins[nextFree] = pin;
	lowestValues[nextFree] = lowestValue;
	highestValues[nextFree] =highestValue;
	nextFree++;
}

/**Barrier calibration
@param seconds - Calibration duration in seconds
*/
void Barriers::calibrate(uint16_t seconds) {
	print("Calibration: interrupt the light ray and then remove the obstacle...", true);
	if (nextFree == 0) 
		print("No barriers. First add at least one.", true);
	else {
		eepromRead();
		for (int i = 0; i < nextFree; i++) {
			lowestValues[i] = 32000;
			highestValues[i] = 0;
		}
		unsigned long startMs = millis();
		while (millis() - startMs < seconds * 1000) {
			for (int i = 0; i < nextFree; i++) {
				int value = analogRead(pins[i]);
				if (value < lowestValues[i])
					lowestValues[i] = value;
				if (value > highestValues[i])
					highestValues[i] = value;
			}
		}
		eepromWrite();
		print("Done.", true);
	}
}

/**
EEPROM read
*/
void Barriers::eepromRead() {
	if (nextFree == 0)
		print("No barrier. First add at least one.", true);
	else {
		int eeAddress = TOP_EEPROM_ADDRESS_BARRIER;

		print("Barrier minimums from EEPROM: ");

		for (int i = 0; i < nextFree; i++) {
			uint8_t highByte = EEPROM.read(eeAddress++);
			uint8_t lowByte = EEPROM.read(eeAddress++);
			int value = (highByte << 8) | lowByte;
			lowestValues[i] = value;
			if (i != 0) 
				print(", ");
			print(value);
		}
		print("", true);

		print("Barrier maximums from EEPROM: ");
		for (int i = 0; i < nextFree; i++) {
			uint8_t highByte = EEPROM.read(eeAddress++);
			uint8_t lowByte = EEPROM.read(eeAddress++);
			int value = (highByte << 8) | lowByte;
			highestValues[i] = value;
			if (i != 0)
				print(", ");
			print(value);
		}
		print("", true);
	}
}

/**
Writing into EEPROM
*/
void Barriers::eepromWrite() {
	int eeAddress = TOP_EEPROM_ADDRESS_BARRIER;

	print("Writing minimums to EEPROM: ");

	for (int i = 0; i < nextFree; i++) {
		uint8_t highByte = lowestValues[i] >> 8;
		uint8_t lowByte = lowestValues[i] & 0xFF;
		EEPROM.write(eeAddress++, highByte);
		EEPROM.write(eeAddress++, lowByte);
		if (i != 0)
			print(", ");
		print(lowestValues[i]);
	}
	print("", true);

	print("Writing maximums to EEPROM: ");

	for (int i = 0; i < nextFree; i++) {
		uint8_t highByte = highestValues[i] >> 8;
		uint8_t lowByte = highestValues[i] & 0xFF;
		int adr = eeAddress;
		EEPROM.write(eeAddress++, highByte);
		EEPROM.write(eeAddress++, lowByte);
		if (i != 0) 
			print(", ");

		highByte = EEPROM.read(adr++);
		lowByte = EEPROM.read(adr++);
		int value = (highByte << 8) | lowByte;
		if (value != highestValues[i])
			error("Error writing into EEPROM.");

		print(highestValues[i]);
	}
	print("", true);
}


/**Barrier interrupted or not
@param sensorNumber - Sensor number. Each call of function add() assigns a increasing number to the sensor.
@return - If interrupted, true.
*/
bool Barriers::interrupt(uint8_t sensorNumber) {
	bool itIs = analogRead(pins[sensorNumber]) < (lowestValues[sensorNumber] + highestValues[sensorNumber]) * 0.5;
	if (itIs)
		lastTimeInterrupted = millis();
	return itIs;
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Barriers::print(String message, bool eol) {
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

/**Test
@param numericValues - If true, display analog readings. If not, digital (X is interrupt).
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Barriers::test(bool numericValues, BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			if (numericValues) {
				print(analogRead(pins[i]));
				print(" ");
			}
			else {
				if (interrupt(i)) 
					print("X");
				else 
					print(".");
			}
		}
		print("", true);

		delay(200);
	}
}


/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Barriers::Barriers(HardwareSerial * hardwareSerial)
{
	serial = hardwareSerial;
	nextFree = 0;
}


Barriers::~Barriers(){}

#include "UltrasonicSRF08s.h"

/** Add a SRF08
@param address - An address the SRF08 uses.
@param enabled - Enabled
*/
void UltrasonicSRF08s::add(uint8_t address, bool isEnabled) {
	if (nextFree >= MAX_ULTRASONIC_SRF08)
		error("Too many SRF08 sensors.");

	addresses[nextFree] = address;
	enabled[nextFree] = isEnabled;
	lastReadMs[nextFree] = 0;
	nextFree++;
}

/** Distance in cm, uses polling. The program must wait for a return wave - very slow.
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@return - Distance.
*/
uint16_t UltrasonicSRF08s::distance(uint8_t sensorNumber) {
	if (!enabled[sensorNumber])
		return 0;
	pulse(sensorNumber);
	return echo(sensorNumber);
}

/** Distance in cm, asynchronous. The program will not wait for a return wave - very fast.
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@param cacheMaxMs - Maximum cache time, starting when triggered.
@return - Distance.
*/
uint16_t UltrasonicSRF08s::distanceAsync(uint8_t sensorNumber, uint32_t cacheMaxMs) {
	if (!enabled[sensorNumber])
		return 0;
	const bool VERBOSE = false;
	const uint32_t cacheMinMs = 80;
	if (lastReadMs[sensorNumber] == 0 || millis() - lastReadMs[sensorNumber]  > cacheMaxMs) {//Cache too old, fire, wait and then read.
		if (VERBOSE)
			print(" Cache miss, limit: " + (String)cacheMaxMs + ", now: " + (String)(millis() - lastReadMs[sensorNumber]) + " ms.", true);
		pulse(sensorNumber);
		lastDistance[sensorNumber] = echo(sensorNumber);
		lastReadMs[sensorNumber] = millis();
	}
	else if (millis() - lastPulseMs[sensorNumber] > cacheMinMs) {//Time between cacheMinMs and cacheMaxMs. Echo should be ready so read it first. Use cache for distance but send a new pulse.
		//Do  not change lastDistance - already OK.
		unsigned long startMs = millis();
		lastDistance[sensorNumber] = echo(sensorNumber);
		if (VERBOSE && millis() - startMs > 1)
			print(" Reading not ready, limit: " + (String)1 + ", now: " + (String)(millis() - startMs) + " ms.", true);
		pulse(sensorNumber);
	} 
	else { //Time less than cacheMinMs - echo not available. Do nothing. To early to send a new pulse. Cached distance is already OK.

	}
	return lastDistance[sensorNumber];
}

/** By using echo values, calculate distance in cm
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@return - Distance
*/
uint16_t UltrasonicSRF08s::echo(uint8_t sensorNumber) {
	if (!enabled[sensorNumber])
		return 0;
	if (sensorNumber >= nextFree)
		error("SRF08 does not exist.");
	uint32_t ms = millis();
	while (true) {
		Wire.beginTransmission(addresses[sensorNumber]);  // Begin communication with the SRF module
		Wire.write(0x00);                                 // Sends the command byte, when this byte is read it returns the software revision
		Wire.endTransmission();

		Wire.requestFrom((int)addresses[sensorNumber], 1);     // Request 1 byte
		if (Wire.available() > 0) {						  // While byte available
			int software = Wire.read();                   // Get byte
			software = software;
			break;
		}

		if (millis() - ms > 300)
			error("Ultrasonic timeout");
	}

	Wire.beginTransmission(addresses[sensorNumber]);      // start communicating with SRFmodule
	Wire.write(0x02);									  // Call the register for start of ranging data
	Wire.endTransmission();

	Wire.requestFrom((int)addresses[sensorNumber], 2);         // Request 2 bytes from SRF module
	while (Wire.available() < 2);						  // Wait for data to arrive
	byte highByte = Wire.read();                          // Get high byte
	byte lowByte = Wire.read();                           // Get low byte
	lastReadMs[sensorNumber] = millis();

	uint16_t d = (highByte << 8) + lowByte;                // Put them together
	return d == 0 ? 0xFFFF : d;
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void UltrasonicSRF08s::print(String message, bool eol) {
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

/** Trigger a sound wave
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
*/
void UltrasonicSRF08s::pulse(uint8_t sensorNumber) {
	if (!enabled[sensorNumber])
		return;
	Wire.beginTransmission(addresses[sensorNumber]); // Start communticating with SRF08
	Wire.write(0x00);                                // Send Command Byte
	Wire.write(0x51);                                // Send 0x51 to start a ranging
	Wire.endTransmission();
	lastPulseMs[sensorNumber] = millis();
}

/** Set range
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@param maxGain - Maximum amplifier gain, 0x00 - 0x1F
@param range - Defines time the sensor waits for return wave: (range + 1) * 43 mm
*/
void UltrasonicSRF08s::rangeAndGain(uint8_t sensorNumber, uint8_t maxGain, uint8_t range) {
	if (!enabled[sensorNumber])
		return;
	uint8_t bytes[] = { 0x01, maxGain };
	Wire.beginTransmission(addresses[sensorNumber]);
	Wire.write(bytes, 2);// Max. Gain Register (default 31). Max. gain 103
	Wire.endTransmission();

	//Range
	bytes[0] = 0x02;
	bytes[1] = range;
	Wire.beginTransmission(addresses[sensorNumber]);
	Wire.write(bytes, 2); //Range 43 mm + 4 * 43 mm
	Wire.endTransmission();
}

/** Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
@param async - Asynchronous mode.
*/
void UltrasonicSRF08s::test(BreakCondition breakWhen, bool async) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			if (!enabled[i])
				continue;
			int d = async ? distanceAsync(i) : distance(i);
			print((String)d + " cm ");
		}
		print("", true);
		delay(200);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
UltrasonicSRF08s::UltrasonicSRF08s(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

UltrasonicSRF08s::~UltrasonicSRF08s(){}

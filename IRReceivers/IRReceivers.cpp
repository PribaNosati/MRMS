#include "IRReceivers.h"

/**Add a sensor
@param pin - Digital pin the sensor uses
@param angleDegrees - Angle in degrees. Robot front is 0 degrees and positive angles are to the right.
*/
void IRReceivers::add(byte pin, double angleDegrees)
{
	if (nextFree >= MAX_IR_RECEIVERS)
		error("Too many IR receivers");
	pinMode(pin, INPUT);
	pins[nextFree] = pin;
	angles[nextFree] = angleDegrees;
	nextFree++;
}

/** Does a light source exist (like an RCJ IR ball) or not?
@return - Exists or not.
*/
bool IRReceivers::anyIRSource(uint16_t threshold) {
	for (int i = 0; i < nextFree; i++)
		if (cumulatives[i] > threshold)
			return true;
	return false;
}

/** Index of a sensor left to the current.
@param currentIndex - index of the current sensor.
@return - index of a left sensor.
*/
uint8_t IRReceivers::indexLeft(uint8_t currentIndex) {
	if (currentIndex == 0)
		return nextFree - 1;
	else
		return currentIndex - 1;
}

/** Index of a sensor right to the current.
@param currentIndex - index of the current sensor.
@return - index of a right sensor.
*/
uint8_t IRReceivers::indexRight(uint8_t currentIndex) {
	if (currentIndex == nextFree - 1)
		return 0;
	else
		return currentIndex + 1;
}

/** Angle of the light source (like an RCJ IR ball)
@return - Angle in degrees. Robot front is 0 degrees and positive angles are to the right.
*/
IRSource IRReceivers::irSource() {
	//First find 3 adjacent sensors with the most readings.
	int maxValue = -1;
	int indexForMaxValue = 0;
	for (int i = 0; i < nextFree; i++) {
		float currValue = cumulatives[i] + cumulatives[indexLeft(i)] + cumulatives[indexRight(i)];//Current + 2 adjacent
		if (currValue > maxValue) {
			maxValue = currValue;
			indexForMaxValue = i;
		}
	}

	IRSource source;
	float nominator = 0;
	float denominator = 0;
	bool add360Degrees = false;
	if (indexForMaxValue != -1) {
		for (int i = -3; i <= 3; i++) {

			int shiftedIndex = indexForMaxValue + i;
			if (shiftedIndex < 0)
				shiftedIndex += nextFree;
			if (shiftedIndex >= nextFree) 
				shiftedIndex -= nextFree;

			if (i!= 2 && shiftedIndex == 0)
				add360Degrees = true;

			float angle = angles[shiftedIndex];
			if (add360Degrees)
				angle += 360;
			nominator += angle * cumulatives[shiftedIndex];
			denominator += cumulatives[shiftedIndex];

		}

		if (abs(denominator) < 0.01)
			source.any = false;
		else {
			float ratio = nominator / denominator;
			if (ratio < -180)
				ratio += 360;
			else if (ratio > 180)
				ratio -= 360;
			source.angle = ratio;
			source.any = true;
		}
	}
	else {
		source.angle = 0;
		source.any = false;
	}
	return source;
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void IRReceivers::print(String message, bool eol) {
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
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
@param updateByTimerInterrupts - If so, no update() will be called in the function.
*/
void IRReceivers::test(BreakCondition breakWhen, bool updateByTimerInterrupts) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			print((String)(int)angles[i] + "D:");
			char buffer[SAMPLE_LENGTH > 99 ? 3 : 2];
			sprintf(buffer, "%2d", cumulatives[i]);
			print(buffer);
			print("   ");
		}

		IRSource source = irSource();
		if (source.any) {
			float direction = source.angle;
			print(" source: " + (String)round(direction) + " deg");
		}
		else
			print(" No source.");

		print("", true);

		uint32_t startMs = millis();
		while (millis() - startMs < 200)
			if (!updateByTimerInterrupts)
				update();
	}
}

/** Periodically updates internal cumulatives.
*/
void IRReceivers::update() {
	for (int i = 0; i < nextFree; i++)
		if (!digitalRead(pins[i])) {
			if (cumulatives[i] < 2)
			cumulatives[i]++;
		}
		else
			if (cumulatives[i] > 0)
				cumulatives[i]--;
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
IRReceivers::IRReceivers(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
	for (int i = 0; i < MAX_IR_RECEIVERS; i++)
		cumulatives[i] = 0;
}

IRReceivers::~IRReceivers(){}

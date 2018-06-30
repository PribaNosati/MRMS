#include "ReflectanceSensors.h"
#include <EEPROM.h>

/** Constructor
@param autoCalibration - 0 - no autocalibration. Any other number - the robot will be decreasing minimums and increasing maximums during its run, according to the read values.
@param percentageForLine - A number between 0 and 1. If a percentage (a ratio: [currentValue - minimumValue / maximumValue - minimumValue) is bigger
than percentageForLine, the ratio will influence line's center calculation.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication.
*/
ReflectanceSensors::ReflectanceSensors(int autoCalibrationValue, double percentForLineValue, HardwareSerial * hardwareSerial)
{
	autoCalibration = autoCalibrationValue;
	percentForLine = percentForLineValue;
	nextFree = 0;
	serial = hardwareSerial;
}

/** Add a sensor
@param pin - Analog pin the sensor uses.
@param mm - Distances in millimeters from a longitudinal axis of the robot, for a line follower. Negative values are for left sensors.
@param lowestValue - Sensor's lowest values.
@param highestValue - Sensor's highest value.
@param forLineCalculation - If true, the sensor will be used in a calculation that finds line's center (for a line follower).
*/
void ReflectanceSensors::add(byte pin, double mm, bool forLineCalculation , uint16_t lowestValue, uint16_t highestBrightValue, uint16_t highestExtraBrightValue)
{
	if (nextFree >= MAX_REFLECTANCE_SENSORS)
		error("Too many reflectance sensors");
	pins[nextFree] = pin;
	mms[nextFree] = mm;
	darkValues[nextFree] = autoCalibration ? 9999 : lowestValue;
	brightValues[nextFree] = autoCalibration ? 0 : highestBrightValue;
	extraBrightValues[nextFree] = autoCalibration ? 0 : highestExtraBrightValue;
	forLineCalculations[nextFree] = forLineCalculation;
	nextFree++;
}

/** Any sensor reads a bright surface, like white.
@return - Any or not.
*/
bool ReflectanceSensors::anyBright() {
	for (int i = 0; i < nextFree; i++)
		if (!isDark(i))
			return true;
	return false;
}

/** Line calibration
@param seconds - Calibration will last so long.
@param calibrationType - BRIGHT_AND_DARK: moving over dark and bright surfaces. DARK - only average dark. BRIGHT - only average bright. 
	Similar for EXTRA_BRIGHT (like a mirror or a metal foil).
*/
void ReflectanceSensors::calibrate(uint16_t seconds, CalibrationType calibrationType) {
	print("Calibration...", true);
	if (nextFree == 0) 
		print("No reflective sensors. First add at least one.", true);
	else {
		eepromRead(calibrationType);
		if (calibrationType == BRIGHT_AND_DARK)
			for (int i = 0; i < nextFree; i++) {
				darkValues[i] = 32000;
				brightValues[i] = 0;
			}
		unsigned long startMs = millis();
		while (millis() - startMs < seconds * 1000) {
			for (int i = 0; i < nextFree; i++) {
				int value = analogRead(pins[i]);
				switch (calibrationType) {
				case BRIGHT_AND_DARK:
					if (value < darkValues[i])
						darkValues[i] = value;
					if (value > brightValues[i])
						brightValues[i] = value;
					break;
				case DARK: //Average
				case BRIGHT:
				case EXTRA_BRIGHT:
					//extraBrightValues[i] = extraBrightValues[i] * i / (i + 1) + value / (i + 1);
					valueSet(calibrationType, i, valueGet(calibrationType, i) * i / (i + 1) + value / (i + 1));
					break;
				default:
					error("Wrong type");
					break;
				}
			}
		}
		if (calibrationType == BRIGHT_AND_DARK || calibrationType == DARK)
			eepromWrite(DARK);
		if (calibrationType == BRIGHT_AND_DARK || calibrationType == BRIGHT)
			eepromWrite(BRIGHT);
		if (calibrationType == EXTRA_BRIGHT)
			eepromWrite(EXTRA_BRIGHT);
		print("Done.", true);
	}
}

/** Display calibration data
@param sensorNumber - Sensor's ordinal number, the one that is used for calibration.
*/
void ReflectanceSensors::calibrationDisplay(uint8_t sensorNumber) {
	print("Dark: " + String(darkValues[sensorNumber]) + " Bright: " + String(brightValues[sensorNumber]) + " Extra bright: " + 
		String(extraBrightValues[sensorNumber]) + ".", true);
}

/** Sensor count
@return - count.
*/
int ReflectanceSensors::count()
{
	return nextFree;
}

/** Detection of a faulty sensor - not in use yet.
*/
void ReflectanceSensors::detectFaulty()
{
	//todo
	for (int i = 0; i < nextFree; i++) {

	}
}

/**	Reading from EEPROMa
@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright.
*/
void ReflectanceSensors::eepromRead(CalibrationType calibrationType) {
	if (nextFree == 0) 		
		print("No reflective sensors. First add at least one.", true);
	else {
		if (calibrationType == ALL) {
			eepromRead(DARK);
			eepromRead(BRIGHT);
			eepromRead(EXTRA_BRIGHT);
		}
		else if (calibrationType == BRIGHT_AND_DARK) {
			eepromRead(DARK);
			eepromRead(BRIGHT);
		}
		else {
			print("Reading " + name(calibrationType) + " from EEPROM: ");

			int eeAddress = eepromStartAddress(calibrationType);
			for (int i = 0; i < nextFree; i++) {
				uint8_t highByte = EEPROM.read(eeAddress++);
				uint8_t lowByte = EEPROM.read(eeAddress++);
				int value = (highByte << 8) | lowByte;
				valueSet(calibrationType, i, value);
				if (i != 0) 
					print(", ");
				print(value);
			}
			print("", true);
		}
	}
}

/**	EEPROM start address
@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright (like for a mirror).
@return - address
*/
uint16_t ReflectanceSensors::eepromStartAddress(CalibrationType calibrationType) {
	switch (calibrationType)
	{
	case DARK:
		return TOP_EEPROM_ADDRESS_REFLECTANCE;
	case BRIGHT:
		return TOP_EEPROM_ADDRESS_REFLECTANCE + nextFree * 2;
	case EXTRA_BRIGHT:
		return TOP_EEPROM_ADDRESS_REFLECTANCE + nextFree * 4;
	case BRIGHT_AND_DARK:
	default:
		error("Wrong calibration, type: " + (String)calibrationType);
		return 0;
	}
}

/**	Writing into EEPROM
@param calibrationType - BRIGHT_AND_DARK both. DARK - only dark. BRIGHT - only bright. EXTRA_BRIGHT - only extra bright.
*/
void ReflectanceSensors::eepromWrite(CalibrationType calibrationType) {
	print("Writing " + name(calibrationType) + " to EEPROM: ");

	int eeAddress = eepromStartAddress(calibrationType);
	for (int i = 0; i < nextFree; i++) {
		uint8_t highByte = valueGet(calibrationType, i) >> 8;
		uint8_t lowByte = valueGet(calibrationType, i) & 0xFF;
		EEPROM.write(eeAddress++, highByte);
		EEPROM.write(eeAddress++, lowByte);
		if (i != 0) {
			Serial.print(", ");
			if (serial != 0)
				serial->print(", ");
		}
		print(valueGet(calibrationType, i));
	}
	print("", true);
}

/** Finding a (black) line
@param lineFound - At least one sensor detected the line, output parameter.
@param nonLineFound -  At least one sensor did not detect the line, output parameter.
@return - Line's center distance, in millimeters, measuring form the robot's longitudinal axis, if found. If not, the last center.
*/
double ReflectanceSensors::findLine(bool & lineFound, bool & nonLineFound) {
	lineFound = false;
	nonLineFound = false;
	static double lastCenterMm = 0;
	const bool WHITE_IS_HIGH_VALUE = true;

	//The line's center is calculated by using analog readings and sensors positions as a mean value.
	double nominator = 0;
	double denominator = 0;
	for (int i = 0; i < count(); i++) {
		if (!forLineCalculations[i])
			continue;
		double reading = analogRead(pins[i]);

		if (autoCalibration) {// Autocalibration increases maximums and decreases minimums.
			if (reading > brightValues[i])
				brightValues[i] = reading;
			if (reading < darkValues[i])
				darkValues[i] = reading;
		}

		double percentLine;

		percentLine = (WHITE_IS_HIGH_VALUE ? brightValues[i] - reading : reading - darkValues[i]) / (brightValues[i] - darkValues[i]);
		if (percentLine > percentForLine) {
			nominator += percentLine * mms[i];
			denominator += percentLine;
		}

		if (percentLine > 0.5)
			lineFound = true;
		if (percentLine < 0.5)
			nonLineFound = true;
	}

	double centerMm = 0;
	if (lineFound && fabsf(denominator) > 0.01) 
		centerMm = nominator / (double)denominator;

	if (lineFound) {
		lastCenterMm = centerMm;
		return centerMm;
	}
	else
		return lastCenterMm;
}

/** Sensor reads a dark surface (like black).
@param sensorNumber - Sensor's index. Function add() assigns 0 to first LED, 1 to second, etc.
@param verbose - verbose output.
@return - It is dark or not.
*/
bool ReflectanceSensors::isDark(byte sensorNumber, bool verbose) {
	uint16_t reading = analogRead(pins[sensorNumber]);
	uint16_t limit =  darkValues[sensorNumber] * (1 - FACTOR_FOR_DARK) + brightValues[sensorNumber] * FACTOR_FOR_DARK;
	bool itIs = reading < limit;
	if (verbose) 
		print(" Dark: " + (String)reading + "<" + (String)limit + "? " + (itIs ? "Yes." : "No.") + ", values " + (String)darkValues[sensorNumber]+
			"-" + (String)brightValues[sensorNumber] + ".", true);
	if (!itIs)
		lastTimeAnyBrightMs = millis();
	return itIs;
}

/** Sensor reads a very bright surface (like a mirror or a shiny metal).
@param sensorNumber - Sensor's index. Function add() assigns 0 to first LED, 1 to second, etc.
@return - It is bright or not.
*/
bool ReflectanceSensors::isExtraBright(byte sensorNumber) {
	bool itIs = analogRead(pins[sensorNumber]) > (extraBrightValues[sensorNumber] + brightValues[sensorNumber]) * 0.5;
	return itIs;
}

/** Name
@return - Name
*/
String ReflectanceSensors::name(CalibrationType calibrationType) {
	switch (calibrationType)
	{
	case DARK:
		return "dark";
	case BRIGHT:
		return "bright";
	case EXTRA_BRIGHT:
		return "extra bright";
	case BRIGHT_AND_DARK:
	default:
		error("Wrong calibration type");
		return "";
	}
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void ReflectanceSensors::print(String message, bool eol) {
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

/**
@return - Raw reading
*/
uint16_t ReflectanceSensors::raw(uint8_t sensorNumber) {
	return  analogRead(pins[sensorNumber]);
}

/**Test
@param numericValues - If true, displays analog values. If not, digital (X for a dark line, a space otherwise).
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void ReflectanceSensors::test(bool numericValues, BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < count(); i++) {
			if (numericValues) {
				print(analogRead(pins[i]));
				print(" ");
			}
			else {
				if (isDark(i)) 
					print("X");
				else 
					print(".");
			}
		}
		print("", true);

		delay(200);
	}
}

	/**	Single value, dark, bright, or extra bright
	@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright.
	@return - Value
	*/
uint16_t ReflectanceSensors::valueGet(CalibrationType calibrationType, uint8_t index) {
	switch (calibrationType)
	{
	case DARK:
		return darkValues[index];
	case BRIGHT:
		return brightValues[index];
	case EXTRA_BRIGHT:
		return extraBrightValues[index];
	case BRIGHT_AND_DARK:
	default:
		error("Wrong calibration val.");
		return 0;
	}
}

/**	Set single value, dark, bright, or extra bright
@param calibrationType - DARK - dark. BRIGHT - bright. EXTRA_BRIGHT - extra bright.
@param value - Value to be set
*/
void ReflectanceSensors::valueSet(CalibrationType calibrationType, uint8_t index, uint16_t value) {
	switch (calibrationType)
	{
	case DARK:
		darkValues[index] = value;
		break;
	case BRIGHT:
		brightValues[index] = value;
		break;
	case EXTRA_BRIGHT:
		extraBrightValues[index] = value;
		break;
	case BRIGHT_AND_DARK:
	default:
		error("Wrong calibration: " + (String)calibrationType);
	}
}

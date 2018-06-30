#include "IRDistanceSensors.h"
#include <EEPROM.h>

/**Add a sensor
@param pin - Analog pin the sensor uses.
@param enabled - Enabled
*/
void IRDistanceSensors::add(byte pin, bool isEnabled)
{
	if (nextFree >= MAX_IR_DISTANCE_SENSORS)
		error("Too many distance sensors");
	pins[nextFree] = pin;
	enabled[nextFree] = isEnabled;
	nextFree++;
	if (isEnabled && !calibrated)
		eepromRead();
}

/** Calibration, distances in cm for given analog values. 2 modes: with external calibration function supplied or without. If
no function supplied, the obstacle will have to be moved by hand to exact distances. If supplied, the obstacles can be
moved slowly between minimum (3 cm) and maximum (60 cm) distances. A typical external function returns
distances measured by an ultrasonic sensor or a LIDAR, at the same position, and pointing in the same direction, as this
reflectance sensor.
@param sensorNumber - Sensor's ordinal number, the one that is used for calibration (an obstacle must be in front of that one).
@param externalCalibrationFunction - External function returning exact distance.
*/
void IRDistanceSensors::calibrate(uint8_t sensorNumber, float(*externalCalibrationFunction)()) {
	if (!enabled[sensorNumber])
		return;
	if (externalCalibrationFunction == 0) {
		print("When done, type something.", true);
		for (int i = 0; i < pointCount; i++) {

			print("Put an obstacle at " + (String)cm[i] + " cm.");

			if (serial != 0)
				while (!Serial.available() && !serial->available());
			else
				while (!Serial.available())
					;

			reading[i] = analogRead(pins[sensorNumber]);

			print(" Reading: " + (String)reading[i], true);

			while (Serial.available())
				Serial.read();
			if (serial != 0) {
				while (serial->available())
					serial->read();
			}
		}
	}
	else {
		uint16_t remaining[pointCount];
		const float PRECISION = 0.3;
		for (uint8_t i = 0; i < pointCount; i++)
			remaining[i] = cm[i];
		bool anyNew = false;
		uint8_t cnt = 0;
		uint32_t lastMs = 0;
		while (true) {

			//Display remaining
			if (anyNew || cnt % 50 == 0) {
				bool anyLeft = false;
				print("", true);
				print("Move obstacle slowly to the remaining distances:");
				for (uint8_t i = 0; i < pointCount; i++)
					if (remaining[i] != 0) {
						print(" " + (String)remaining[i] + "cm");
						anyLeft = true;
					}
				print("", true);
				if (!anyLeft)
					break;
				anyNew = false;
				cnt = 1;
			}
			else {

				//Watch for an appropriate distance.
				float externalDistance = (*externalCalibrationFunction)();
				for (uint8_t i = 0; i < pointCount; i++) {
					if (remaining[i] != 0 && remaining[i] - PRECISION <= externalDistance && externalDistance <= remaining[i] + PRECISION) {
						remaining[i] = 0;
						reading[i] = analogRead(pins[sensorNumber]);
						anyNew = true;
					}
				}

				if (millis() - lastMs > 200) {
					char buffer[10];
					sprintf(buffer, "%4.1f cm ", externalDistance);
					print(buffer);
					if (cnt++ % 5 == 0)
						print("", true);
					lastMs = millis();
				}
			}
		}
	}

	//Faulty data correction.
	bool any = false;
	for (int i = 0; i < pointCount - 1; i++)
		if (reading[i] < reading[i + 1]) {
			reading[i] = reading[i + 1] + 1;
			any = true;
		}
	if (any)
		print("Readings not reliable.");

	eepromWrite();
	eepromRead(true);
	print("End.", true);
}

/** Display calibration data
@param sensorNumber - Sensor's ordinal number, the one that is used for calibration.
*/
void IRDistanceSensors::calibrationDisplay(uint8_t sensorNumber) {
	if (!enabled[sensorNumber])
		return;
	for (uint8_t i = 0; i < pointCount; i++) {
		if (i != 0)
			print(", ");
		print((String)cm[i] +" cm: " + (String)reading[i]);
	}
	print("", true);
}

/** Set calibration
@param readings - instead of EEPROM values, use external values for calibration
*/
void IRDistanceSensors::calibrationSet(uint16_t readings[]) {
	for (uint8_t i = 0; i < pointCount; i++)
		reading[i] = readings[i];
}

/**Number of sensors
@return - count.
*/
int IRDistanceSensors::count()
{
	return nextFree;
}

/**Distance
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - Distance in cm.
*/
float IRDistanceSensors::distance(byte sensorNumber) {
	if (!enabled[sensorNumber])
		return 0;
	uint16_t value = analogRead(pins[sensorNumber]);
	uint16_t distance = 0;
	if (value >= reading[0])
		distance = cm[0];
	else if (value < reading[pointCount - 1])
		distance = cm[pointCount - 1];
	else
		for (int i = 0; i < pointCount - 1; i++)
			if (value < reading[i] && value >= reading[i + 1])
				distance = map(value, reading[i], reading[i + 1], cm[i], cm[i + 1]);
	return distance;
}

/**
Reading from EEPROMa
@param verbose - If true, displays the read values.
*/
void IRDistanceSensors::eepromRead(bool verbose) {
	if (nextFree == 0)
		print("There are no IR sensors. Add at least one.", true);
	else {
		int eeAddress = TOP_EEPROM_ADDRESS_IR_DISTANCE;

		if (verbose)
			print("Reading from EEPROM: ");

		for (int i = 0; i < nextFree; i++) {
			uint8_t highByte = EEPROM.read(eeAddress++);
			uint8_t lowByte = EEPROM.read(eeAddress++);
			int value = (highByte << 8) | lowByte;
			reading[i] = value;
			if (verbose) {
				if (i != 0)
					print(", ");
				print(value);
			}
		}
		if (verbose)
			print("", true);
	}
}

/**
Writing into EEPROM
*/
void IRDistanceSensors::eepromWrite() {
	int eeAddress = TOP_EEPROM_ADDRESS_IR_DISTANCE;

	print("Writing to EEPROM: ");

	for (int i = 0; i < pointCount; i++) {
		uint8_t highByte = reading[i] >> 8;
		uint8_t lowByte = reading[i] & 0xFF;
		int adr = eeAddress;
		EEPROM.write(eeAddress++, highByte);
		EEPROM.write(eeAddress++, lowByte);
		if (i != 0)
			print(", ");

		highByte = EEPROM.read(adr++);
		lowByte = EEPROM.read(adr++);
		int value = (highByte << 8) | lowByte;
		if (value != reading[i])
			error("Error writing into EEPROM.");

		print(reading[i]);
	}
	print("", true);
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void IRDistanceSensors::print(String message, bool eol) {
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
@param numericValues - If true, numeric (analog) values will be displayed. If not, distance in cm.
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void IRDistanceSensors::test(bool numericValues, BreakCondition breakWhen) {
	char buffer[4];
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < count(); i++) {
			if (!enabled[i])
				continue;
			if (numericValues) {
				sprintf(buffer, "%3i", analogRead(pins[i]));
				print(buffer);
				print(" ");
			}
			else {
				if (i != 0)
					print(" ");
				sprintf(buffer, "%3i", (int)round(distance(i)));
				print(buffer);
			}
		}
		print("", true);

		delay(200);
	}
}

/**Constructor
@param useCalibrationFromEEPROM - If true, read calibration data from EEPROM. In that case calibration (calibrate()) must be run at least once. The results are stored premanently.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
IRDistanceSensors::IRDistanceSensors(bool useCalibrationFromEEPROM, HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
	calibrated = !useCalibrationFromEEPROM;//If no calibration used, mark as calibrated.

	cm[0] = DISTANCE_1_IN_CM;
	cm[1] = DISTANCE_2_IN_CM;
	cm[2] = DISTANCE_3_IN_CM;
	cm[3] = DISTANCE_4_IN_CM;
	cm[4] = DISTANCE_5_IN_CM;
	cm[5] = DISTANCE_6_IN_CM;
	cm[6] = DISTANCE_7_IN_CM;
	cm[7] = DISTANCE_8_IN_CM;
	cm[8] = DISTANCE_9_IN_CM;
	cm[9] = DISTANCE_10_IN_CM;
	cm[10] = DISTANCE_11_IN_CM;
	cm[11] = DISTANCE_12_IN_CM;

	reading[0] = IR_READING_FOR_1;
	reading[1] = IR_READING_FOR_2;
	reading[2] = IR_READING_FOR_3;
	reading[3] = IR_READING_FOR_4;
	reading[4] = IR_READING_FOR_5;
	reading[5] = IR_READING_FOR_6;
	reading[6] = IR_READING_FOR_7;
	reading[7] = IR_READING_FOR_8;
	reading[8] = IR_READING_FOR_9;
	reading[9] = IR_READING_FOR_10;
	reading[10] = IR_READING_FOR_11;
	reading[11] = IR_READING_FOR_12;
}

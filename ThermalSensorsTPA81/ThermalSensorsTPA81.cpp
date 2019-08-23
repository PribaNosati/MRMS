#include "ThermalSensorsTPA81.h"

/** Add a TPA81
@param address - The sensor's I2C address
*/
void ThermalSensorsTPA81::add(uint8_t address) {
	if (nextFree >= MAX_THERMAL_SENSORS_TPA81)
		error("Too many TPA81s.");

	addresses[nextFree] = address;
	nextFree++;
}

/** Read a sensor
@param senzorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@param reg - Register. Check Devantech's documentation.
@return - Result of reading.
*/
byte ThermalSensorsTPA81::readSensor(byte sensorNumber, byte reg) { // Function to receive one byte of data from TPA81
	Wire.beginTransmission(addresses[sensorNumber]);                // Begin communication with TPA81
	Wire.write(reg);											// Send reg to TPA81
	Wire.endTransmission();
	Wire.requestFrom((int)addresses[sensorNumber], 1);          // Request 1 byte
	while (Wire.available() < 1);								// Wait for byte to arrive
	byte data = Wire.read();									// Get byte
	return(data);												// return byte
}

/** Read a temperature.
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@param rayNumber - Ray's number, 0 to 7.
@return - Temperature in degrees Celsius.
*/
int ThermalSensorsTPA81::temperature(uint8_t sensorNumber, uint8_t rayNumber)
{
	if (rayNumber > 7)
		error("Ray index out of range");
	return readSensor(sensorNumber, rayNumber + 2);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void ThermalSensorsTPA81::test(BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			for (int j = 0; j < 8; j++) {
				if (j != 0) {
					Serial.print(" ");
					if (serial != 0) serial->print(" ");
				}
				Serial.print(temperature(i, j));
				if (serial != 0) serial->print(temperature(i, j));
			}
			Serial.print("   ");
			if (serial != 0) serial->print("   ");
		}
		Serial.println();
		if (serial != 0) serial->println();
		delay(200);
	}
}

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
ThermalSensorsTPA81::ThermalSensorsTPA81(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

ThermalSensorsTPA81::~ThermalSensorsTPA81() {}

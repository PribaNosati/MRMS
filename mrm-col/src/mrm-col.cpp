#include "mrm-col.h"

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication. Example:
Mrm_col(&Serial2);
*/
Mrm_col::Mrm_col(HardwareSerial * hardwareSerial)
{
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_col::~Mrm_col() {}

/**Add a sensor
@param i2cBus - I2C bus 0 is Wire, 1 is Wire1, etc. For example, command for Wire1 is: add(&Wire1);
@param address - I2C adress
*/
void Mrm_col::add(uint8_t address, uint8_t integrationTime, uint8_t dataConversionType, uint8_t gain)
{
	if (nextFree >= MAX_MRM_COL)
		error("Too many sensors");
	print("AS7262...");
	addresses[nextFree] = address;

	uint8_t hwVersion = read(nextFree, I2C_AS726X_HW_VERSION);
	if (hwVersion != 0x3E)
		error("Wrong HW version: " + (String)(int)hwVersion);

	dataConversionTypeSet(nextFree, dataConversionType);
	gainSet(nextFree, gain);
	integrationTimeSet(nextFree, integrationTime);
	ledIndicatorSet(nextFree, false);

	nextFree++;
	print("OK", true);
}

/** Blue
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - blue
*/
uint16_t Mrm_col::blue(uint8_t sensorNumber) {
	return(channel(sensorNumber, I2C_AS7262_B));
}

/** Calibrated blue
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - calibrated blue
*/
float Mrm_col::blueCalibrated(uint8_t sensorNumber) {
	return(calibrated(sensorNumber, I2C_AS7262_B_CAL));
}

/** Returns calibrated color channel
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param firstRegister
@return - calibrated channel
*/
float Mrm_col::calibrated(uint8_t sensorNumber, uint8_t firstRegister)
{
	union Mix {
		uint8_t bytes[4];
		float aFloat;
	};
	Mix mix;

	mix.bytes[3] = read(sensorNumber, firstRegister + 0);
	mix.bytes[2] = read(sensorNumber, firstRegister + 1);
	mix.bytes[1] = read(sensorNumber, firstRegister + 2);
	mix.bytes[0] = read(sensorNumber, firstRegister + 3);

	return mix.aFloat;
}

/** Color channel
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param firstRegister
@return - channel
*/
uint16_t Mrm_col::channel(uint8_t sensorNumber, uint8_t firstRegister)
{
	return (read(sensorNumber, firstRegister) << 8) | read(sensorNumber, firstRegister + 1);
}

/** Sets data conversion type
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param conversionType:
	0, continuous reading of violet, blue, green and yellow 
	1, continuous reading of green, yellow, orange and red
	2, continuous reading of all channels (power-on default but requires 2 x integration time, double compared to types 0 or 1)
	3, one-time reading of all channels
*/
void Mrm_col::dataConversionTypeSet(uint8_t sensorNumber, uint8_t conversionType)
{
	if (conversionType > 3) 
		conversionType = 3;

	uint8_t value = (read(sensorNumber, I2C_AS726X_CONTROL_SETUP) & 0b11110011) | (conversionType << 2); 
	write(sensorNumber, I2C_AS726X_CONTROL_SETUP, value);
}



/** Data ready?
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - ready or not
*/
boolean Mrm_col::dataReadyGet(uint8_t sensorNumber)
{
	return read(sensorNumber, I2C_AS726X_CONTROL_SETUP) & (1 << 1);
}

/** Clear data-ready flag
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_col::dataReadyClear(uint8_t sensorNumber)
{
	uint8_t value = read(sensorNumber, I2C_AS726X_CONTROL_SETUP) & 0b11111101;
	write(sensorNumber, I2C_AS726X_CONTROL_SETUP, value);
}



/** Set gain
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param gain:
	0, 1x (default)
	1, 3.7x
	2, 16x
	3, 64x
*/
void Mrm_col::gainSet(uint8_t sensorNumber, uint8_t gain)
{
	if (gain > 3) 
		gain = 3;

	uint8_t value = (read(sensorNumber, I2C_AS726X_CONTROL_SETUP) & 0b11001111) | (gain << 4); 
	write(sensorNumber, I2C_AS726X_CONTROL_SETUP, value);
}

/** Green
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - green
*/
uint16_t Mrm_col::green(uint8_t sensorNumber) {
	return(channel(sensorNumber, I2C_AS7262_G));
}

/** Calibrated green
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - calibrated green
*/
float Mrm_col::greenCalibrated(uint8_t sensorNumber) {
	return(calibrated(sensorNumber, I2C_AS7262_G_CAL));
}

/** Set integration time
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param value - integration time will be value x 2.8 ms. value is between 0 and 255 (default). 
*/
void Mrm_col::integrationTimeSet(uint8_t sensorNumber, uint8_t value)
{
	write(sensorNumber, I2C_AS726X_INT_T, value); //Write
}

/** Enable/disable interrupt pin's output
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param enabled - true (enabled) or false (disabled, default value)
*/
void Mrm_col::interruptSet(uint8_t sensorNumber, bool enabled)
{
	uint8_t value = (read(sensorNumber, I2C_AS726X_CONTROL_SETUP) & 0b10111111) | (enabled << 7);
	write(sensorNumber, I2C_AS726X_CONTROL_SETUP, value);
}

/** LED for illumination, set current
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param current:
0, 12.5 mA (default)
1, 25 mA
2, 50 mA
3, 100 mA
*/
void Mrm_col::ledForIlluminationCurrentSet(uint8_t sensorNumber, uint8_t current)
{
	if (current > 3)
		current = 3;
	uint8_t value = (read(sensorNumber, I2C_AS726X_LED_CONTROL) & 0b11001111) | (current << 4);
	write(sensorNumber, I2C_AS726X_LED_CONTROL, value);
}

/** LED for illumination on-off
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param enabled - true (enabled) or false (disabled, default)
*/
void Mrm_col::ledForIlluminationSet(uint8_t sensorNumber, bool enabled)
{
	uint8_t value = (read(sensorNumber, I2C_AS726X_LED_CONTROL) & 0b11110111) | (enabled << 3);
	write(sensorNumber, I2C_AS726X_LED_CONTROL, value);
}

/** LED indicator's current
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param current:
	0, 1 mA (default)
	1, 2 mA
	2, 4 mA
	3, 8 mA
*/
void Mrm_col::ledIndicatorCurrentSet(uint8_t sensorNumber, uint8_t current)
{
	if (current > 3) 
		current = 3;
	uint8_t value = (read(sensorNumber, I2C_AS726X_LED_CONTROL) & 0b11111001) | (current << 1); 
	write(sensorNumber, I2C_AS726X_LED_CONTROL, value); 
}

/** LED indicator on-off
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param enabled - true (enabled) or false (disabled, default)
*/
void Mrm_col::ledIndicatorSet(uint8_t sensorNumber, bool enabled)
{
	uint8_t value = (read(sensorNumber, I2C_AS726X_LED_CONTROL) & 0b11111110) | enabled;
	write(sensorNumber, I2C_AS726X_LED_CONTROL, value);
}

/** Orange
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - orange
*/
uint16_t Mrm_col::orange(uint8_t sensorNumber) {
	return(channel(sensorNumber, I2C_AS7262_O));
}

/** Calibrated orange
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - calibrated orange
*/
float Mrm_col::orangeCalibrated(uint8_t sensorNumber) {
	return(calibrated(sensorNumber, I2C_AS7262_O_CAL));
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void Mrm_col::print(String message, bool eol) {
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

/** Read a virtual register
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param virtualReg - register address
@return - virtual register's content
*/
uint8_t Mrm_col::read(uint8_t sensorNumber, uint8_t virtualReg)
{
	volatile uint8_t status, d;
	while (1)
	{
		// Read slave I²C status to see if the read buffer is ready.
		status = readRegister(sensorNumber, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
			// No inbound TX pending at slave. Okay to write now.
			break;
		//delay(AS7262_POLLING_DELAY);
	}
	// Send the virtual register address (setting bit 7 to indicate a pending write).
	writeRegister(sensorNumber, I2C_AS72XX_SLAVE_WRITE_REG, virtualReg);
	while (1)
	{
		// Read the slave I²C status to see if our read data is available.
		status = readRegister(sensorNumber, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0)
			// Read data is ready.
			break;
		//delay(AS7262_POLLING_DELAY);
	}
	// Read the data to complete the operation.
	d = readRegister(sensorNumber, I2C_AS72XX_SLAVE_READ_REG);
	return d;
}

/** I2C register read
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param address
@return - register's content
*/
uint8_t Mrm_col::readRegister(uint8_t sensorNumber, uint8_t address)
{
	Wire.beginTransmission((int)addresses[sensorNumber]);
	Wire.write(address);
	Wire.endTransmission();
	Wire.requestFrom((int)addresses[sensorNumber], 1);
	if (Wire.available()) {
		return (Wire.read());
	}
	else {
		Serial.println("I2C Error");
		return (0xFF); //Error
	}
}

/** Red
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - red
*/
uint16_t Mrm_col::red(uint8_t sensorNumber) {
	return(channel(sensorNumber, I2C_AS7262_R));
}

/** Calibrated red
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - calibrated red
*/
float Mrm_col::redCalibrated(uint8_t sensorNumber) {
	return(calibrated(sensorNumber, I2C_AS7262_R_CAL));
}

/** Reset
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void Mrm_col::reset(uint8_t sensorNumber)
{
	uint8_t value = read(sensorNumber, I2C_AS726X_CONTROL_SETUP) | (1 << 7);
	write(sensorNumber, I2C_AS726X_CONTROL_SETUP, value);
}

/**Temperature in C
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - temperature
*/
uint8_t Mrm_col::temperature(uint8_t sensorNumber){
	return read(sensorNumber, I2C_AS726X_DEVICE_TEMP);
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_col::test(BreakCondition breakWhen) {
	char buffer[100];
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			sprintf(buffer, "Vio:%4i/%4i Blu:%4i/%4i Gre:%4i/%4i Yel:%4i/%4i Ora:%4i/%4i Red:%4i/%4i Temp:%i", violet(i), (int)violetCalibrated(i),
				blue(i), (int)blueCalibrated(i), green(i), (int)greenCalibrated(i), yellow(i), (int)yellowCalibrated(i), orange(i), 
				(int)orangeCalibrated(i), red(i), (int)redCalibrated(i), (int)temperature(i));
			Serial.print(buffer);
		}
		print("", true);

		delay(200);
	}
}

/** Violet
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - violet
*/
uint16_t Mrm_col::violet(uint8_t sensorNumber) {
	return(channel(sensorNumber, I2C_AS7262_V));
}

/** Calibrated violet
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - calibrated violet
*/
float Mrm_col::violetCalibrated(uint8_t sensorNumber) {
	return(calibrated(sensorNumber, I2C_AS7262_V_CAL));
}

/** Write to a virtual register
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param virtualReg - register address
@data - new virtual register's content
*/
void Mrm_col::write(uint8_t sensorNumber, uint8_t virtualReg, uint8_t data)
{
	volatile uint8_t status;
	while (1)
	{
		// Read slave I²C status to see if the write buffer is ready.
		status = readRegister(sensorNumber, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
			// No inbound TX pending at slave. Okay to write now.
			break;
	}
	// Send the virtual register address (setting bit 7 to indicate a pending write).
	writeRegister(sensorNumber, I2C_AS72XX_SLAVE_WRITE_REG, (virtualReg | 0x80));
	while (1)
	{
		// Read the slave I²C status to see if the write buffer is ready.
		status = readRegister(sensorNumber, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
			// No inbound TX pending at slave. Okay to write data now.
			break;
	}
	// Send the data to complete the operation.
	writeRegister(sensorNumber, I2C_AS72XX_SLAVE_WRITE_REG, data);
}

/** I2C register write
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param address
@param value - new register's content
*/
void Mrm_col::writeRegister(uint8_t sensorNumber, uint8_t address, uint8_t value)
{
	Wire.beginTransmission((int)addresses[sensorNumber]);
	Wire.write(address);
	Wire.write(value);
	Wire.endTransmission();
}

/** Yellow
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - yellow
*/
uint16_t Mrm_col::yellow(uint8_t sensorNumber) {
	return(channel(sensorNumber, I2C_AS7262_Y));
}

/** Calibrated yellow
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - calibrated yellow
*/
float Mrm_col::yellowCalibrated(uint8_t sensorNumber) {
	return(calibrated(sensorNumber, I2C_AS7262_Y_CAL));
}
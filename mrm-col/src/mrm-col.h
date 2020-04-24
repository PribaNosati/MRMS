#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
Purpose: using MRMS mrm-col sensor
@author MRMS team
@version 0.0 2018-08-13
Licence: You can use this code any way you like.
*/

#define MAX_MRM_COL 6 //Maximum number of sensors. 

#define I2C_AS72XX_SLAVE_STATUS_REG 0x00
#define I2C_AS72XX_SLAVE_WRITE_REG 0x01
#define I2C_AS72XX_SLAVE_READ_REG 0x02
#define I2C_AS72XX_SLAVE_TX_VALID 0x02
#define I2C_AS72XX_SLAVE_RX_VALID 0x01

#define I2C_AS726X_DEVICE_TYPE 0x00
#define I2C_AS726X_HW_VERSION 0x01
#define I2C_AS726X_CONTROL_SETUP 0x04
#define I2C_AS726X_INT_T 0x05
#define I2C_AS726X_DEVICE_TEMP 0x06
#define I2C_AS726X_LED_CONTROL 0x07

#define I2C_AS7262_V 0x08
#define I2C_AS7262_B 0x0A
#define I2C_AS7262_G 0x0C
#define I2C_AS7262_Y 0x0E
#define I2C_AS7262_O 0x10
#define I2C_AS7262_R 0x12
#define I2C_AS7262_V_CAL 0x14
#define I2C_AS7262_B_CAL 0x18
#define I2C_AS7262_G_CAL 0x1C
#define I2C_AS7262_Y_CAL 0x20
#define I2C_AS7262_O_CAL 0x24
#define I2C_AS7262_R_CAL 0x28

typedef bool(*BreakCondition)();

class Mrm_col
{
	int nextFree;
	uint8_t addresses[MAX_MRM_COL];// I2C address
	HardwareSerial * serial; // Additional serial port

	/** Returns calibrated color channel
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param firstRegister
	@return - calibrated channel
	*/
	float calibrated(uint8_t sensorNumber, uint8_t firstRegister);

	/** Color channel
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param firstRegister
	@return - channel
	*/
	uint16_t channel(uint8_t sensorNumber, uint8_t firstRegister);

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);

	/** Read a virtual register
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param virtualReg - register address
	@return - virtual register's content
	*/
	uint8_t read(uint8_t sensorNumber, uint8_t virtualReg);

	/** I2C register read
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param address
	@return - register's content
	*/
	uint8_t readRegister(uint8_t sensorNumber, uint8_t addr);

	/** Write to a virtual register
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param virtualReg - register address
	@data - new virtual register's content
	*/
	void write(uint8_t sensorNumber, uint8_t virtualReg, uint8_t d);
	
	/** I2C register write
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param address
	@param value - new register's content
	*/
	void writeRegister(uint8_t sensorNumber, uint8_t addr, uint8_t val);

public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication. Example:
		Mrm_col(&Serial2);
	*/
	Mrm_col(HardwareSerial * hardwareSerial = 0);

	~Mrm_col();

	/**Add a sensor
	@param i2cBus - I2C bus 0 is Wire, 1 is Wire1, etc. For example, command for Wire1 is: add(&Wire1);
	@param address - I2C adress
	@param integrationTime
	@param dataConversionType
	@param gain
	*/
	void add(byte address = 0x49, uint8_t integrationTime = 50, uint8_t dataConversionType = 2, uint8_t gain = 3);

	/** Blue
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - blue
	*/
	uint16_t blue(uint8_t sensorNumber);

	/** Calibrated blue
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - calibrated blue
	*/
	float blueCalibrated(uint8_t sensorNumber);

	/** Sets data conversion type
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param conversionType:
	0, continuous reading of violet, blue, green and yellow
	1, continuous reading of green, yellow, orange and red
	2, continuous reading of all channels (power-on default)
	3, one-time reading of all channels
	*/
	void dataConversionTypeSet(uint8_t sensorNumber, uint8_t mode);

	/** Data ready?
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - ready or not
	*/
	boolean dataReadyGet(uint8_t sensorNumber);

	/** Clear data-ready flag
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void dataReadyClear(uint8_t sensorNumber);

	/** Set gain
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param gain:
	0, 1x (default)
	1, 3.7x
	2, 16x
	3, 64x
	*/
	void gainSet(uint8_t sensorNumber, uint8_t gain);

	/** Green
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - green
	*/
	uint16_t green(uint8_t sensorNumber);

	/** Calibrated green
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - calibrated green
	*/
	float greenCalibrated(uint8_t sensorNumber);

	/** Set integration time
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param value - integration time will be value x 2.8 ms. value is between 0 and 255 (default).
	*/
	void integrationTimeSet(uint8_t sensorNumber, uint8_t integrationValue);

	/** Enable/disable interrupt pin's output
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param enabled - true (enabled) or false (disabled, default value)
	*/
	void interruptSet(uint8_t sensorNumber, bool enabled);

	/** LED for illumination, set current
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param current:
	0, 12.5 mA (default)
	1, 25 mA
	2, 50 mA
	3, 100 mA
	*/
	void ledForIlluminationCurrentSet(uint8_t sensorNumber, uint8_t current);

	/** LED for illumination on-off
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param enabled - true (enabled) or false (disabled, default)
	*/
	void ledForIlluminationSet(uint8_t sensorNumber, bool enabled);

	/** LED indicator's current
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param current:
	0, 1 mA (default)
	1, 2 mA
	2, 4 mA
	3, 8 mA
	*/
	void ledIndicatorCurrentSet(uint8_t sensorNumber, uint8_t current);

	/** LED indicator on-off
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@param enabled - true (enabled) or false (disabled, default)
	*/
	void ledIndicatorSet(uint8_t sensorNumber, bool enabled);

	/** Orange
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - orange
	*/
	uint16_t orange(uint8_t sensorNumber);

	/** Calibrated orange
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - calibrated orange
	*/
	float orangeCalibrated(uint8_t sensorNumber);

	/** Red
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - red
	*/
	uint16_t red(uint8_t sensorNumber);

	/** Calibrated red
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - calibrated red
	*/
	float redCalibrated(uint8_t sensorNumber);

	/** Reset
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	*/
	void reset(uint8_t sensorNumber);

	/**Temperature in C
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - temperature
	*/
	uint8_t temperature(uint8_t sensorNumber);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);

	/** Violet
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - violet
	*/
	uint16_t violet(uint8_t sensorNumber);

	/** Calibrated violet
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - calibrated violet
	*/
	float violetCalibrated(uint8_t sensorNumber);

	/** Yellow
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - yellow
	*/
	uint16_t yellow(uint8_t sensorNumber);

	/** Calibrated yellow
	@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
	@return - calibrated yellow
	*/
	float yellowCalibrated(uint8_t sensorNumber);
};

//Declaration of error function. Definition is in Your code.
void error(String message);


#include "mrm-lid1.h"
#include <Arduino.h>

/** Add a sensor. It assigns sensor 0 to the first sensor, 1 to the second, etc. This number ("sensorNumber") is used to
call other functions for this sensor.
@param pin - Sensor's enable pin. @param pin - Sensor's enable pin. 0xFF - not used. Sensor will be enabled if this pin if left
unconnected due to internal pull-up.
@param i2c_addr - I2C address. 0x29 must not be used to any other I2C device, even if not used for any VL53L1X.
*/
void VL53L0Xs::add(uint8_t pin, uint8_t i2c_addr) {
	if (nextFree >= MAX_VL53L0XS)
		error("Too many lidars.");

	if (pin != 0xFF) {
		pins[nextFree] = pin;
		pinMode(pins[nextFree], OUTPUT);
	}
	digitalWrite(pins[nextFree], LOW);

	lastDistance[nextFree] = 0;
	lastMeasurement[nextFree] = 0;

	pDev[nextFree] = new VL53L0X_Dev_t();

	pDev[nextFree]->I2cDevAddr = i2c_addr;
	pDev[nextFree]->comms_type = 1;
	pDev[nextFree]->comms_speed_khz = 400;

	nextFree++;
}

/** Starts sensors. It must be called after all the add() calls.
@param continuousMeasurement - Non-stop measuring.
@param verbose - Detailed display.
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
*/
void VL53L0Xs::begin(bool continuousMeasurement, boolean verbose, uint8_t sensorNumber){
	continuous = continuousMeasurement;
	if (sensorNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			begin(continuousMeasurement, verbose, i);
	else{
		print("VL53L0X " + (String)sensorNumber + "...");

		uint8_t i2cAddress = pDev[sensorNumber]->I2cDevAddr;
		pDev[sensorNumber]->I2cDevAddr = 0x29;

		pinMode(pins[sensorNumber], INPUT);

		//Only once after power-up.
		if (VL53L0X_DataInit(pDev[sensorNumber]) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Init");

		//Set unique I2C address
		VL53L0X_SetDeviceAddress(pDev[sensorNumber], i2cAddress << 1);
		pDev[sensorNumber]->I2cDevAddr = i2cAddress;

		//Device info
		VL53L0X_DeviceInfo_t DeviceInfo;
		if (VL53L0X_GetDeviceInfo(pDev[sensorNumber], &DeviceInfo) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Info");
		if (verbose) 
			print("VL53L0X " + (String)DeviceInfo.Name + ", type: " + (String)DeviceInfo.Type + ", id: " + (String)(int)DeviceInfo.ProductId +
				", " + (String)DeviceInfo.ProductRevisionMajor + "." + (String)DeviceInfo.ProductRevisionMinor, true);

		//Specific settings
		if (VL53L0X_StaticInit(pDev[sensorNumber]) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Static");

		//Reference SPAD calibration. Once during the initialization
		uint32_t  refSpadCount;
		uint8_t   isApertureSpads;
		if (VL53L0X_PerformRefSpadManagement(pDev[sensorNumber], &refSpadCount, &isApertureSpads) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Spad");
		if (verbose) 
			print("refSpadCount = " + (String)refSpadCount + ", isApertureSpads = " + (String)isApertureSpads, true);

		//Ref (temperature) calibration. This function should be run from time to time -todo
		uint8_t VhvSettings, PhaseCal;
		if (VL53L0X_PerformRefCalibration(pDev[sensorNumber], &VhvSettings, &PhaseCal) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Temp");

		// Sigma (TOF), in mm
		if (VL53L0X_SetLimitCheckEnable(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Sigma");	

		// Amplitude of the reflected signal in MCPS
		if (VL53L0X_SetLimitCheckEnable(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Amplitude");

		// Signal minimum threshold
		if (VL53L0X_SetLimitCheckEnable(pDev[sensorNumber], VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Thresh");
		if (VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5 * 0.023 * 65536)) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Fix");

		//Continuous or single measurements
		if (VL53L0X_SetDeviceMode(pDev[sensorNumber], continuous ? VL53L0X_DEVICEMODE_CONTINUOUS_RANGING : VL53L0X_DEVICEMODE_SINGLE_RANGING) != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "Mode");

		if (continuousMeasurement)
			if (VL53L0X_StartMeasurement(pDev[sensorNumber]) != VL53L0X_ERROR_NONE)
				errorVL(sensorNumber, "Start");

		//if (Status != VL53L0X_ERROR_NONE) 
		//	errorVL(sensorNumber, "VL53L0X Error: " + (String)Status);

#if SAMPLE_COUNT_AVG > 0
		averageCounter[sensorNumber] = 0xFF;
#endif

		print("OK", true);
	}
}

/** Distance measurement
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param average - if true, average measurement. Sample count is defined by using SAMPLE_COUNT_AVG.
@return - Distance in mm.
*/
uint16_t VL53L0Xs::distance(uint8_t sensorNumber, bool average) {
	if (sensorNumber >= nextFree)
		error("Range d");

	uint16_t mm;
	if (millis() - lastMeasurement[sensorNumber] < 5)
		mm = lastDistance[sensorNumber];
	else {
		static VL53L0X_RangingMeasurementData_t rangingMeasurementData;

		if (continuous) {
			//uint8_t *ready = 0; - todo
			//if (VL53L0X_GetMeasurementDataReady(pDev[sensorNumber], ready))
			//	errorVL(sensorNumber, "Ready");
			if (VL53L0X_GetRangingMeasurementData(pDev[sensorNumber], &rangingMeasurementData) != VL53L0X_ERROR_NONE)
				errorVL(sensorNumber, "Get");
			//if (VL53L0X_ClearInterruptMask(pDev[sensorNumber], 0) != VL53L0X_ERROR_NONE)
			//	errorVL(sensorNumber, "Clear");
		}
		else {
			if (VL53L0X_PerformSingleMeasurement(pDev[sensorNumber]) != VL53L0X_ERROR_NONE)
				errorVL(sensorNumber, "Single");
			if (VL53L0X_GetRangingMeasurementData(pDev[sensorNumber], &rangingMeasurementData) != VL53L0X_ERROR_NONE)
				errorVL(sensorNumber, "Get");
			if (VL53L0X_ClearInterruptMask(pDev[sensorNumber], 0) != VL53L0X_ERROR_NONE)
				errorVL(sensorNumber, "Clear");
		}

		//if (pRangingMeasurementData->RangeStatus == 4)
		//	print("Out of range");
		mm = rangingMeasurementData.RangeMilliMeter;
		lastDistance[sensorNumber] = mm;
		lastMeasurement[sensorNumber] = millis();
		if (mm == 0xFFFF)
			error("VL53L0X");
	}
	if (average) {
#if SAMPLE_COUNT_AVG > 0
		if (averageCounter[sensorNumber] == 0xFF) {
			for (uint8_t i = 0; i < SAMPLE_COUNT_AVG; i++)
				averageSamples[sensorNumber][i] = mm;
			averageCounter[sensorNumber] = 0;
		}
		else {
			averageSamples[sensorNumber][averageCounter[sensorNumber]] = mm;
			if (++averageCounter[sensorNumber] >= SAMPLE_COUNT_AVG)
				averageCounter[sensorNumber] = 0;
		}
		uint32_t sum = 0;
		for (uint8_t i = 0; i < SAMPLE_COUNT_AVG; i++)
			sum += averageSamples[sensorNumber][i];
		return sum / (float)SAMPLE_COUNT_AVG;
#else
		error("SAMPLE_COUNT_AVG must be > 0");
		return 0;
#endif
	}
	else
		return mm;
}

/** Error-handling function
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param message - error message.
*/
void VL53L0Xs::errorVL(uint8_t sensorNumber, String message) {
	if (sensorNumber >= nextFree)
		error("Range e");
	VL53L0X_State *state = 0;
	VL53L0X_GetPalState(pDev[sensorNumber], state);
	//char errorString[100];
	//VL53L0X_GetPalErrorString(*state, errorString);
	error(message + " Sensor " + (String)sensorNumber + ", state " + (String)(*state));// +": " + errorString);
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void VL53L0Xs::print(String message, bool eol) {
	Serial.print(message);
	if (serial)
		serial->print(message);
	if (eol) {
		Serial.println();
		if (serial)
			serial->println();
	}
}

/** Ranging profile
@param value - profile
@param sensorNumber -  Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
0xFF - all sensors.
*/
void VL53L0Xs::rangeProfileSet(RangeProfile value, uint8_t sensorNumber) {
	if (sensorNumber == 0xFF) {
		for (uint8_t i = 0; i < nextFree; i++)
			rangeProfileSet(value, i);
	}
	else {
		if (sensorNumber >= nextFree)
			error("Range f");
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		switch (value) {
		case DefaultMode:
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 200000);//Overall timing budget, min. 20 ms, default 33 ms
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.40 * 65536);
			break;
		case HighAccuracy:
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 200000);
			break;
		case LongRange:
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 33000);
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetVcselPulsePeriod(pDev[sensorNumber], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetVcselPulsePeriod(pDev[sensorNumber], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
			break;
		case HighSpeed:
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetLimitCheckValue(pDev[sensorNumber], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32 * 65536));
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 20000);
			break;
		}
		if (Status != VL53L0X_ERROR_NONE)
			errorVL(sensorNumber, "RangeProfile");
	}
}

/** Reset
@param sensorNumber - sensor number. 0xFF - all the sensors.
*/
void VL53L0Xs::reset(uint8_t sensorNumber) {
	if (sensorNumber == 0xFF)
		for (int i = 0; i < nextFree; i++)
			reset(sensorNumber);
	else {
		print(" Reset VL53L0X " + (String)sensorNumber + ".");
		if (sensorNumber >= nextFree)
			error("Range t");
		VL53L0X_ResetDevice(pDev[sensorNumber]);
		begin(continuous, false, sensorNumber);
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
@param average - if true, average measurement. Sample count is defined by using SAMPLE_COUNT_AVG.
*/
void VL53L0Xs::test(BreakCondition breakWhen, bool average)
{
	char buffer[10];
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			if (i != 0) 
				print(" ");
			sprintf(buffer, "%4i", (int)round(distance(i, average)));
			print(buffer);
		}
		print("", true);
		delay(100);
	}
}

/** Constructor
@param hardwareSerial - optional additional serial port, for example for Bluetooth.
*/
VL53L0Xs::VL53L0Xs(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
}
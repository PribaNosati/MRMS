#include "mrm-lid2.h"

/** Add a sensor. It assigns sensor 0 to the first sensor, 1 to the second, etc. This number ("sensorNumber") is used to
call other functions for this sensor.
@param pin - Sensor's enable pin. @param pin - Sensor's enable pin. 0xFF - not used. Sensor will be enabled if this pin if left
unconnected due to internal pull-up.
@param i2c_addr - I2C address. 0x29 must not be used to any other I2C device, even if not used for any VL53L1X.
*/
void VL53L1Xs::add(uint8_t pin, uint8_t i2c_addr) {
	if (nextFree >= MAX_VL53L1XS)
		error("Too many lidars.");

	pins[nextFree] = pin;
	if (pin != 0xFF) {
		pinMode(pins[nextFree], OUTPUT);
		digitalWrite(pins[nextFree], LOW);
	}

	lastDistance[nextFree] = 0;
	lastMeasurement[nextFree] = 0;

	pDev[nextFree] = new VL53L1_Dev_t();
	pDev[nextFree]->I2cDevAddr = i2c_addr << 1;
	pDev[nextFree]->comms_type = 1;
	pDev[nextFree]->comms_speed_khz = 400;
	nextFree++;
}

/** Starts sensors. It must be called after all the add() calls.
@param verbose - Detailed display.
*/
void VL53L1Xs::begin(boolean verbose) {
	for (uint8_t i = 0; i < nextFree; i++) {
		Serial.print("VL53L1X " + (String)i + "...");
		uint8_t i2cAddress = pDev[i]->I2cDevAddr;
		pDev[i]->I2cDevAddr = 0x52; //0x29 x 2
		if (pins[i] != 0xFF)
			pinMode(pins[i], INPUT);//Enable the sensor by activating internal pull-up.

		//VL53L1_software_reset(pDev[i]); //?

		if (VL53L1_WaitDeviceBooted(pDev[i]) != VL53L1_ERROR_NONE)
			errorVL(i, "Wait");
		//Set unique I2C address
		if (pins[i] != 0xFF) {
			VL53L1_SetDeviceAddress(pDev[i], i2cAddress);
			pDev[i]->I2cDevAddr = i2cAddress;
		}
		//Only once after power-up.
		if (VL53L1_DataInit(pDev[i]) != VL53L1_ERROR_NONE)
			errorVL(i, "Init");
		//Device info
		VL53L1_DeviceInfo_t DeviceInfo;
		if (VL53L1_GetDeviceInfo(pDev[i], &DeviceInfo) != VL53L1_ERROR_NONE)
			errorVL(i, "Info");
		if (verbose) {
			Serial.println("VL53L1X " + (String)DeviceInfo.Name + ", type: " + (String)DeviceInfo.Type + ", id: " + (String)(int)DeviceInfo.ProductId + 
				", " + (String)DeviceInfo.ProductRevisionMajor + "." + (String)DeviceInfo.ProductRevisionMinor);
		}

		//Specific settings
		if (VL53L1_StaticInit(pDev[i]) != VL53L1_ERROR_NONE)
			errorVL(i, "Static");

#define CALIBRATE false
#if CALIBRATE
		//Reference SPAD calibration. Once during the initialization
		if (VL53L1_PerformRefSpadManagement(pMyDevice[i]) != VL53L1_ERROR_NONE)
			errorVL(i, "Spad");

		//Offset calibration
		if (VL53L1_PerformOffsetCalibration(pMyDevice[i], 10) != VL53L1_ERROR_NONE)
			errorVL(i, "Offset");

		//Cross talk
		if (VL53L1_PerformSingleTargetXTalkCalibration(pMyDevice[i], 10) != VL53L1_ERROR_NONE)
			errorVL(i, "Xtalk");

		//
		VL53L1_CalibrationData_t *pCalibrationData;
		if (VL53L1_GetCalibrationData(pMyDevice[i], pCalibrationData) != VL53L1_ERROR_NONE)
			errorVL(i, "Cali");
#endif

		//Distance mode
		if (VL53L1_SetDistanceMode(pDev[i], VL53L1_DISTANCEMODE_SHORT) != VL53L1_ERROR_NONE)
			errorVL(i, "DMod");

		//Timinig budget, 20 to 1000 ms. 1000 is 1 ms.
		if (VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev[i], 20000) != VL53L1_ERROR_NONE)
			errorVL(i, "Budg");

		//Inter-measurement time. Timing budget (prev. parameter) is a good value for this one:
		if (VL53L1_SetInterMeasurementPeriodMilliSeconds(pDev[i], 50) != VL53L1_ERROR_NONE)
			errorVL(i, "Inter"); // reduced to 50 ms from 500 ms in ST example

		if (VL53L1_StartMeasurement(pDev[i]) != VL53L1_ERROR_NONE)
			errorVL(i, "Start");

		if (pins[i] != 0xFF)
			digitalWrite(pins[nextFree], LOW);//Disable again.

		Serial.println("OK");
	}

	//Enable all
	for (uint8_t i = 0; i < nextFree; i++) 
		if (pins[i] != 0xFF)
			pinMode(pins[i], INPUT);
}

/** Distance measurement
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@return - Distance in mm.
*/
uint16_t VL53L1Xs::distance(uint8_t sensorNumber) {
	if (sensorNumber >= nextFree)
		error("Range d");

	if (millis() - lastMeasurement[sensorNumber] < 5)
		return lastDistance[sensorNumber];
	else {
		uint16_t mm = 0;

		static VL53L1_RangingMeasurementData_t RangingData;

		//uint8_t * pMeasuremenDataReady;
		//if (VL53L1_GetMeasurementDataReady(pDev[sensorNumber], pMeasuremenDataReady) != VL53L1_ERROR_NONE)
		//	errorVL(sensorNumber, "Rdy");

		//if (VL53L1_WaitMeasurementDataReady(pDev[sensorNumber]) != VL53L1_ERROR_NONE)
		//	errorVL(sensorNumber, "Wai");

		if (VL53L1_GetRangingMeasurementData(pDev[sensorNumber], &RangingData) != VL53L1_ERROR_NONE)
			errorVL(sensorNumber, "Get");
		mm = RangingData.RangeMilliMeter;
		if (VL53L1_ClearInterruptAndStartMeasurement(pDev[sensorNumber]) != VL53L1_ERROR_NONE)
			errorVL(sensorNumber, "Clr");

		switch (RangingData.RangeStatus) {
			case VL53L1_RANGESTATUS_RANGE_VALID: // Ranging measurement is valid
				lastDistance[sensorNumber] = mm;
				break;
			case VL53L1_RANGESTATUS_SIGMA_FAIL: // Raised if sigma estimator check is above the internal defined threshold
				warning("sigma fail");
				break;
			case VL53L1_RANGESTATUS_SIGNAL_FAIL: // Raised if signal value is below the internal defined threshold
				warning("signal fail");
				break;
			case VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL: // Raised when phase is out of bounds
				warning("out of bounds");
				break;
			case VL53L1_RANGESTATUS_HARDWARE_FAIL: // Raised in case of HW or VCSEL failure
				warning("HW fail");
				break;
			case VL53L1_RANGESTATUS_WRAP_TARGET_FAIL: // Wrapped target, not matching phases
				warning("wrapped target");
				break;
			case VL53L1_RANGESTATUS_PROCESSING_FAIL: // Internal algorithm underflow or overflow
				warning("algorithm");
				break;
			case VL53L1_RANGESTATUS_RANGE_INVALID: // The reported range is invalid.
				warning("range invalid");
				break;
			default:
				warning("failure");
				break;
		}
		lastMeasurement[sensorNumber] = millis();
		return mm;
	}
}

/** Error-handling function
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param message - error message.
*/
void VL53L1Xs::errorVL(uint8_t sensorNumber, String message) {
	if (sensorNumber >= nextFree)
		error("Range e");
	VL53L1_State *state = 0;
	VL53L1_GetPalState(pDev[sensorNumber], state);
	//char errorString[100];
	//VL53L1_GetPalErrorString(*state, errorString);
	error(message + " Sensor " + (String)sensorNumber + ", state " + (String)(*state));// +": " + errorString);
}

/** Ranging profile
@param value - profile
@param sensorNumber -  Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
0xFF - all sensors.
*/
void VL53L1Xs::rangeProfileSet(RangeProfile value, uint8_t sensorNumber) {
	if (sensorNumber == 0xFF) {
		for (uint8_t i = 0; i < nextFree; i++)
			rangeProfileSet(value, i);
	}
	else {
		if (sensorNumber >= nextFree)
			error("Range f");
		VL53L1_Error Status = VL53L1_ERROR_NONE;
		switch (value) {
		case ShortRange:
			//Distance mode
			if (VL53L1_SetDistanceMode(pDev[sensorNumber], VL53L1_DISTANCEMODE_SHORT) != VL53L1_ERROR_NONE)
				errorVL(sensorNumber, "DMod");
			//Timinig budget, 20 to 1000 ms. 1000 is 1 ms.
			if (VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 20000) != VL53L1_ERROR_NONE)
				errorVL(sensorNumber, "Budg");
		case MidRange:
			//Distance mode
			if (VL53L1_SetDistanceMode(pDev[sensorNumber], VL53L1_DISTANCEMODE_MEDIUM) != VL53L1_ERROR_NONE)
				errorVL(sensorNumber, "DMod");
			//Timinig budget, 20 to 1000 ms. 1000 is 1 ms.
			if (VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 50000) != VL53L1_ERROR_NONE)
				errorVL(sensorNumber, "Budg");
			break;
		case LongRange:
			//Distance mode
			if (VL53L1_SetDistanceMode(pDev[sensorNumber], VL53L1_DISTANCEMODE_LONG) != VL53L1_ERROR_NONE)
				errorVL(sensorNumber, "DMod");
			//Timinig budget, 20 to 1000 ms. 1000 is 1 ms.
			if (VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev[sensorNumber], 100000) != VL53L1_ERROR_NONE)
				errorVL(sensorNumber, "Budg");
			break;
		}
		if (Status != VL53L1_ERROR_NONE)
			errorVL(sensorNumber, "RangeProfile");
	}
}

/** Region of interest, can used to set FOV (field of view). The minimum ROI size is 4x4, maximum (and default) 16x16.
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0.
@param TopLeftX - top left X of the FOV, 0-15.
@param TopLeftY - top left X of the FOV, 0-15.
@param BottomRightX - bottom right X of the FOV, 0-15.
@param BottomRightY - bottom right X of the FOV, 0-15.
*/
void VL53L1Xs::roi(uint8_t sensorNumber, uint8_t TopLeftX, uint8_t TopLeftY, uint8_t BottomRightX, uint8_t BottomRightY) {
	if (sensorNumber == 0xFF) {
		for (uint8_t i = 0; i < nextFree; i++)
			roi(i, TopLeftX, TopLeftY, BottomRightX, BottomRightY);
	}
	else {
		VL53L1_UserRoi_t roiConfig;
		roiConfig.TopLeftX = TopLeftX;
		roiConfig.TopLeftY = TopLeftY;
		roiConfig.BotRightX = BottomRightX;
		roiConfig.BotRightY = BottomRightY;
		if (VL53L1_SetUserROI(pDev[sensorNumber], &roiConfig) != VL53L1_ERROR_NONE)
			errorVL(sensorNumber, "ROI");
	}
}

/** ROI center. Due to assembly tolerances, the optical center of the device can vary.
@param xCenter - returns center's x coordinate, 0-15.
@param yCenter - returns center's y coordinate, 0-15.
*/
void VL53L1Xs::roiCenter(uint8_t sensorNumber, uint8_t &xCenter, uint8_t &yCenter) {
	VL53L1_CalibrationData_t data;
	if (VL53L1_GetCalibrationData(pDev[sensorNumber], &data) != VL53L1_ERROR_NONE)
		errorVL(sensorNumber, "Opt");
	xCenter = data.optical_centre.x_centre;
	yCenter = data.optical_centre.y_centre;
}

/** Stress test
@param breakWhen - A function returning bool, without arguments. If it returns true, the stress() will be interrupted.
*/
void VL53L1Xs::stress(BreakCondition breakWhen) {
	uint32_t startMs = 0;
	uint8_t cnt = 0;
	while (breakWhen == 0 || !(*breakWhen)()) {
		uint8_t sensorNumber = random(0, nextFree);
		if (distance(sensorNumber) > 10000)
			Serial.print("x");
		if (millis() - startMs > 500) {
			startMs = millis();
			Serial.print(".");
			if (cnt++ > 20) {
				cnt = 0;
				Serial.println();
				Serial.print(millis() / 1000);
				Serial.print(" s");
			}
		}
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void VL53L1Xs::test(BreakCondition breakWhen)
{
	char buffer[10];
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			if (i != 0) 
				Serial.print(" ");
			sprintf(buffer, "%4i", (int)round(distance(i)));
			Serial.print(buffer);

		}
		Serial.println();
		delay(50);
	}
}

/** Warning
@param message
*/
void VL53L1Xs::warning(String message){
	if (warnings) {
		message = " VL53L1x WRN: " + message + ".";
		Serial.print(message);
	}
}

/** Constructor
@param hardwareSerial - optional additional serial port, for example for Bluetooth.
@param displayWarnings - display ranging warnings.
*/
VL53L1Xs::VL53L1Xs(HardwareSerial * hardwareSerial, bool displayWarnings) {
	serial = hardwareSerial;
	warnings = displayWarnings;
}

/** Destructor
@param displayWarnings - display ranging warnings.
*/
VL53L1Xs::~VL53L1Xs() {
	for (uint8_t i = 0; i < nextFree; i++)
		VL53L1_StopMeasurement(pDev[i]);
}

#include "mrm-servo.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
*/
Mrm_servo::Mrm_servo(Robot* robot) {
	robotContainer = robot;
	nextFree = 0;
}

Mrm_servo::~Mrm_servo()
{
}

/** Add a servo motor
@param gpioPin - pin number, allowed: 0 - 19, 21 - 23, 25 - 27, 32 - 36, 39
@param deviceName - device's name
@param minDegrees - minimum servo angle
@param maxDegrees - maximum servo angle
@param minDegreesPulseMicroSec - pulse ms for minimum angle
@param maxDegreesPulseMicroSec - pulse ms for maximum angle
@param timerWidth - timer width in bits, 1 - 16. 12 yields angle resolution of about 1º.
*/
void Mrm_servo::add(uint8_t gpioPin, char* deviceName, uint16_t minDegrees, uint16_t maxDegrees, float minDegreesPulseMs, float maxDegreesPulseMs, uint8_t timerWidth)
{
	if (nextFree >= MAX_SERVO_COUNT) {
		strcpy(errorMessage, "Too many servo motors");
		return;
	}

	if (deviceName != 0) {
		if (strlen(deviceName) > 9) {
			strcpy(errorMessage, "Device name too long");
			return;
		}
		strcpy(_name[nextFree], deviceName);
	}

	_timerWidth = timerWidth;
	_minDegrees = minDegrees;
	_maxDegrees = maxDegrees;
	_minDegreesPulseMs = minDegreesPulseMs;
	_maxDegreesPulseMs = maxDegreesPulseMs;


	// Standard servo, 0-180º: 20 ms period, duty 1 - 2 ms. 1.5 ms - neutral position.
	// For pulseWidth=20 ms (50 Hz) and _timerWidth=12, tickLength = (1000 / 50) / (2^12 - 1) = 20/4095 = 0.004884 ms
	// pulseHighWidth = numberOfTicks*tickLength
	// numberOfTicksNeeded = pulseHighWidth/tickLength pulseHighWidthMicroSec/1000000/tickLength = pulseHighWidthMicroSec * f. For 90 degrees numberOfTicksNeeded = 1.5/0.004884 = 307. For 0 degrees numberOfTicksNeeded = 1/0.004884 = 205

	float tickLength = (1000 / (float)MRM_SERVO_FREQUENCY_HZ) / ((1 << _timerWidth) - 1); //tickLength = pulsePeriod/(2^timerWidthBits-1) * 1000, in ms. 
	f = 1 / tickLength;
	
	double resFreq = ledcSetup(nextFree, MRM_SERVO_FREQUENCY_HZ, _timerWidth); // nextFree is channel number, which can be 0 - 15.
	ledcAttachPin(gpioPin, nextFree); // gpioPin assigned to channel nextFree

	nextFree++;
}

void Mrm_servo::sweep() {
	// If variables are not needed in any other function, and  must be persistent, they should be declared static:
	static uint8_t servoDegrees = 90;
	static bool servoIncreasing = true;
	static uint32_t servoLastChangeMs = 0;

	if (millis() - servoLastChangeMs > 12) { //Servo cannot operate faster
		servoDegrees += (servoIncreasing ? 5 : -5);
		if (servoDegrees == 180 || servoDegrees == 0)
			servoIncreasing = !servoIncreasing;
		servoLastChangeMs = millis();
		write(servoDegrees);
	}
}


/**Test
*/
void Mrm_servo::test()
{
	const uint16_t ms = 300; // Min. 12.
	int16_t step = 5;
	int16_t degrees = _minDegrees;
	while (!robotContainer->userBreak()) {
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			write(degrees, deviceNumber);
			print("%i\n\r", degrees);
		}
		robotContainer->delayMs(ms);
		if (degrees + step < _minDegrees || degrees + step > _maxDegrees)
			step = -step;
		degrees += step;
	}

	print("\n\rTest over.\n\r");
	robotContainer->end();
}

/** Move servo
@param degrees - Servo's target angle, 0 - 180º, or 0 - 360°, depending on model, counting clockwise
@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
*/
void Mrm_servo::write( uint16_t degrees, uint8_t servoNumber) {
	if (servoNumber >= nextFree) {
		strcpy(errorMessage, "Servo doesn't exist");
		return;
	}
	degrees = constrain(degrees, _minDegrees, _maxDegrees);
	ledcWrite(servoNumber, map(degrees, _minDegrees, _maxDegrees, _minDegreesPulseMs * f, _maxDegreesPulseMs * f));

	uint16_t _minDegrees;
	uint16_t _minDegreesPulseMicroSec;
	uint16_t _maxDegrees;
	uint16_t _maxDegreesPulseMicroSec;
}

/** Position servo according to user input.
*/
void Mrm_servo::writeInteractive() {
	// Select motor
	print("Enter servo number [0-%i]\n\r", nextFree - 1);
	uint16_t selectedMotor = robotContainer->serialReadNumber(3000, 500, nextFree - 1 > 9, nextFree - 1, false);
	if (selectedMotor != 0xFFFF) {
		print("\n\rTest motor %i\n\r", selectedMotor);

		// Select speed
		bool fixedSpeed = false;
		print("Enter angle \n\r");
		uint16_t degrees = robotContainer->serialReadNumber(2000, 500, false);
		if (degrees != 0xFFFF) {
			write(degrees, selectedMotor);
			print("OK\n\r");
		}
		else
			print("Timeout\n\r");
	}
	else
		print("Timeout\n\r");
}
#include "mrm-servo.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param maxNumberOfServos - Maximum number of servo motors cannot be bigger than 16 since there are 16 PWM channels in ESP32
*/
Mrm_servo::Mrm_servo(Robot* robot, uint8_t maxNumberOfServos) {
	_currentDegrees = new std::vector<uint16_t>(maxNumberOfServos);
	_minDegrees = new std::vector<uint16_t>(maxNumberOfServos);
	_minDegreesPulseMs = new std::vector<float>(maxNumberOfServos);
	_maxDegrees = new std::vector<uint16_t>(maxNumberOfServos);
	_maxDegreesPulseMs = new std::vector<float>(maxNumberOfServos);
	_name = new std::vector<char[10]>(maxNumberOfServos);
	_timerWidth = new std::vector<uint8_t>(maxNumberOfServos);
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
@param timerWidth - timer width in bits, 1 - 16. 12 yields angle resolution of about 1�.
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
		strcpy((*_name)[nextFree], deviceName);
	}

	(*_timerWidth)[nextFree] = timerWidth;
	(*_minDegrees)[nextFree] = minDegrees;
	(*_maxDegrees)[nextFree] = maxDegrees;
	(*_minDegreesPulseMs)[nextFree] = minDegreesPulseMs;
	(*_maxDegreesPulseMs)[nextFree] = maxDegreesPulseMs;

	// Standard servo, 0-180�: 20 ms period, duty 1 - 2 ms. 1.5 ms - neutral position.
	// For pulseWidth=20 ms (50 Hz) and _timerWidth=12, tickLength = (1000 / 50) / (2^12 - 1) = 20/4095 = 0.004884 ms
	// pulseHighWidth = numberOfTicks*tickLength
	// numberOfTicksNeeded = pulseHighWidth/tickLength pulseHighWidthMicroSec/1000000/tickLength = pulseHighWidthMicroSec * f. For 90 degrees numberOfTicksNeeded = 1.5/0.004884 = 307. For 0 degrees numberOfTicksNeeded = 1/0.004884 = 205

	float tickLength = (1000 / (float)MRM_SERVO_FREQUENCY_HZ) / ((1 << timerWidth) - 1); //tickLength = pulsePeriod/(2^timerWidthBits-1) * 1000, in ms. 
	f = 1 / tickLength;
	
	/*double resFreq = */ledcSetup(nextFree, MRM_SERVO_FREQUENCY_HZ, timerWidth); // nextFree is channel number, which can be 0 - 15.
	ledcAttachPin(gpioPin, nextFree); // gpioPin assigned to channel nextFree

	nextFree++;

	write((*_maxDegrees)[nextFree-1] / 2, nextFree - 1);
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
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		int16_t degrees = (*_minDegrees)[deviceNumber];
		while (!robotContainer->userBreak()) {
			
				write(degrees, deviceNumber);
				robotContainer->print("%i\n\r", degrees);

			robotContainer->delayMs(ms);
			if (degrees + step < (*_minDegrees)[deviceNumber] || degrees + step > (*_maxDegrees)[deviceNumber])
				step = -step;
			degrees += step;
		}
	}

	robotContainer->print("\n\rTest over.\n\r");
	robotContainer->end();
}

/** Move servo
@param degrees - Servo's target angle, 0 - 180�, or 0 - 360�, depending on model, counting clockwise
@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
@param ms - Duration of action in ms. 0 ms - immediately.
*/
void Mrm_servo::write( uint16_t degrees, uint8_t servoNumber, uint16_t ms) {
	if (servoNumber >= nextFree) {
		//robotContainer->print("Servo: %i, limit %i failed\n\r", servoNumber, nextFree);
		strcpy(errorMessage, "Servo doesn't exist");
		return;
	}
	degrees = constrain(degrees, (*_minDegrees)[servoNumber], (*_maxDegrees)[servoNumber]);

	ledcWrite(servoNumber, map(degrees, (*_minDegrees)[servoNumber], (*_maxDegrees)[servoNumber], (*_minDegreesPulseMs)[servoNumber] * f, (*_maxDegreesPulseMs)[servoNumber] * f));
	// uint32_t startMs = millis();
	// while (millis() - startMs < ms){
	// 	uint16_t currentDegrees = map(millis(), startMs, startMs + ms, (*_currentDegrees)[servoNumber], degrees);
	// 	ledcWrite(servoNumber, map(currentDegrees, (*_minDegrees)[servoNumber], (*_maxDegrees)[servoNumber], 
	// 		(*_minDegreesPulseMs)[servoNumber] * f, (*_maxDegreesPulseMs)[servoNumber] * f));
	// 	robotContainer->delayMs(10);
	// }
	(*_currentDegrees)[servoNumber] = degrees;

	// uint16_t _minDegrees;
	// uint16_t _minDegreesPulseMicroSec;
	// uint16_t _maxDegrees;
	// uint16_t _maxDegreesPulseMicroSec;
}

/** Position servo according to user input.
*/
void Mrm_servo::writeInteractive() {
	// Select motor
	robotContainer->print("Enter servo number [0-%i]\n\r", nextFree - 1);
	uint16_t selectedMotor = robotContainer->serialReadNumber(3000, 500, nextFree - 1 > 9, nextFree - 1, false);
	if (selectedMotor != 0xFFFF) {
		robotContainer->print("\n\rTest motor %i\n\r", selectedMotor);

		// Select speed
		// bool fixedSpeed = false;
		robotContainer->print("Enter angle \n\r");
		uint16_t degrees = robotContainer->serialReadNumber(2000, 500, false);
		if (degrees != 0xFFFF) {
			write(degrees, selectedMotor);
			robotContainer->print("OK\n\r");
		}
		else
			robotContainer->print("Timeout\n\r");
	}
	else
		robotContainer->print("Timeout\n\r");
}
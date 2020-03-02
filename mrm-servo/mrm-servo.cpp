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
@param timerWidth - timer width in bits, 1 - 16
*/
void Mrm_servo::add(uint8_t gpioPin, char* deviceName, uint8_t timerWidth)
{
	if (nextFree >= MAX_SERVO_COUNT) {
		strcpy(robotContainer->errorMessage, "Too many servo motors");
		return;
	}

	if (deviceName != 0) {
		if (strlen(deviceName) > 9) {
			strcpy(robotContainer->errorMessage, "Device name too long");
			return;
		}
		strcpy(nameThis[nextFree], deviceName);
	}

	timerWidthBits = timerWidth;

	// Servo: 20 ms period, duty 1 - 2 ms. 1.5 ms - neutral position.
#define FREQUENCY_HZ 50
	double resFreq = ledcSetup(nextFree, FREQUENCY_HZ, timerWidthBits); // nextFree is channel number, which can be 0 - 15.
	ledcAttachPin(gpioPin, nextFree); // GPIO 16 assigned to channel 1

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
	const uint8_t ms = 12;
	while (!robotContainer->userBreak()) {

		for (int16_t i = 0; i <= 180; i += 5) {
			write(i);
			robotContainer->print("%i\n\r", i);
			delay(ms);
		}

		for (int16_t i = 180; i >= 0; i -= 5) {
			write(i);
			robotContainer->print("%i\n\r", i);
			delay(ms);
		}
	}

	robotContainer->print("\n\rTest over.\n\r");
	robotContainer->_actionCurrent = NULL;
}

/** Move servo
@param degrees - Servo's target angle, 0 - 180
@param servoNumber - Servo's ordinal number. Each call of function add() assigns a increasing number to the servo, starting with 0.
*/
void Mrm_servo::write( uint16_t degrees, uint8_t servoNumber) {
	if (servoNumber >= nextFree) {
		strcpy(robotContainer->errorMessage, "Servo doesn't exist");
		return;
	}
	degrees = constrain(degrees, 0, 180);
	uint16_t period = (1 << timerWidthBits) - 1;
	ledcWrite(servoNumber, map(degrees, 0, 180, period * 0.025, period * 0.125));// /20 /10
	//ledcWrite(1, map(degrees, 0, 180, PERIOD / 20, PERIOD / 10));
}

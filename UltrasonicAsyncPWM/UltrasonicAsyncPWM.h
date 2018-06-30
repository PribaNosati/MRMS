#pragma once
#include <Arduino.h>

/**
Purpose: using PWM ultrasonic sensors without delays (polling), that means without Arduino pulseIn() function.
@author MRMS team, a part copied from internet
@version 0.2 2018-02-19
Licence: You can use this code any way you like.
*/

#define MAX_ULTRASONIC_ASYNC_PWM 4 // Maximum number of sensors.
#define PULSE_IS_HIGH true // Usually true (pulse is a logic high voltage, like 3.3V, otherwise 0V), but for some sensors is false, like DFRobot URM37.
#define TRIGGER_PULSE_WIDTH 100 // Trigger pulse's width in ms. It could be 100 (ms) but for some sensors 0 is enough.
//#define USE_TIMER_FOR_TRIGGER // If this line is not commented, it will work completely asynchronous, but it consumes timers (and you may need them for 
						//other purposes). Valid only for Teensy. If the TRIGGER_PULSE_WIDTH is 0, You will not need it.

typedef bool(*BreakCondition)();

class UltrasonicAsyncPWM
{
	static void _echo_isr(uint8_t sensorNumber); // Interrupt functions
	static void _echo_isr0();
	static void _echo_isr1();
	static void _echo_isr2();
	static void _echo_isr3();
	static void _echo_isr4();
	static void _echo_isr5();
	static void _echo_isr6();
	static void _echo_isr7();
	static void _timer0();
	static void _timer1();
	static void _timer2();
	static void _timer3();
	static void _timer4();
	static void _timer5();
	static void _timer6();
	static void _timer7();
	uint8_t _echo[MAX_ULTRASONIC_ASYNC_PWM]; // PWM pins, read pulses' widths
	volatile bool _finished[MAX_ULTRASONIC_ASYNC_PWM]; // Results are ready and can be read
	volatile bool _highDetected[MAX_ULTRASONIC_ASYNC_PWM]; // Front edge of impulses arrived.
	volatile uint32_t _impulsEnd[MAX_ULTRASONIC_ASYNC_PWM]; // Times when resulting signals were read from _echo pins.
	volatile uint32_t _impulsStart[MAX_ULTRASONIC_ASYNC_PWM]; // Times when resulting signals reading from _echo pins was started.
	static UltrasonicAsyncPWM* _instance; //Singleton
	uint8_t _trigger[MAX_ULTRASONIC_ASYNC_PWM]; // Trigger pins.
	int nextFree;
	HardwareSerial * serial; //Additional serial port

	/** Print to all serial ports
	@param message
	@param eol - end of line
	*/
	void print(String message, bool eol = false);
	
public:
	/**Constructor
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	*/
	UltrasonicAsyncPWM(HardwareSerial * hardwareSerial = 0);

	/** Add a sensor
	@param trigger - Trigger pin.
	@param echo - Pin for reading echno signal. Not necessary a PWM pin, but it must support interrupts.
	*/
	void add(uint8_t trigger, uint8_t echo);

	/** Distance in cm
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	@return - Distance.
	*/
	float distance(uint8_t sensorNumber);

	/** Is the result ready?
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	*/
	bool isFinished(uint8_t sensorNumber) { return _finished[sensorNumber]; }

	/** A single object. This is not a user function.
	@return - First object of this class.
	*/
	static UltrasonicAsyncPWM* instance() { return _instance; }

	/** Trigger a pulse.
	@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
	*/
	void start(uint8_t sensorNumber);

	/**Test
	@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
	*/
	void test(BreakCondition breakWhen = 0);
};

//Declaration of error function. Definition is in Your code.
void error(const char * message);

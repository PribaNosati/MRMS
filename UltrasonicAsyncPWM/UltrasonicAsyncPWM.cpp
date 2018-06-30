#include "UltrasonicAsyncPWM.h"

//Singleton, not important for a user
UltrasonicAsyncPWM *UltrasonicAsyncPWM::_instance(NULL);

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
UltrasonicAsyncPWM::UltrasonicAsyncPWM(HardwareSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
	if (_instance == 0) _instance = this;//Singleton
}

// This part is important only if a trigger pulse is more than 0 ms long. If a signal is longer, the program will be waiting.
// In general, 2 delays are possible: for a trigger pulse and for a returning pulse.
#ifdef USE_TIMER_FOR_TRIGGER
#if TRIGGER_PULSE_WIDTH != 0
IntervalTimer timer0;// Radi samo za Teensy
IntervalTimer timer1;// Radi samo za Teensy
IntervalTimer timer2;// Radi samo za Teensy
IntervalTimer timer3;// Radi samo za Teensy
IntervalTimer timer4;// Radi samo za Teensy
IntervalTimer timer5;// Radi samo za Teensy
IntervalTimer timer6;// Radi samo za Teensy
IntervalTimer timer7;// Radi samo za Teensy
#endif
#endif

					 // Interrupt functions
void UltrasonicAsyncPWM::_echo_isr0() { _echo_isr(0); }
void UltrasonicAsyncPWM::_echo_isr1() { _echo_isr(1); }
void UltrasonicAsyncPWM::_echo_isr2() { _echo_isr(2); }
void UltrasonicAsyncPWM::_echo_isr3() { _echo_isr(3); }
void UltrasonicAsyncPWM::_echo_isr4() { _echo_isr(4); }
void UltrasonicAsyncPWM::_echo_isr5() { _echo_isr(5); }
void UltrasonicAsyncPWM::_echo_isr6() { _echo_isr(6); }
void UltrasonicAsyncPWM::_echo_isr7() { _echo_isr(7); }
#ifdef USE_TIMER_FOR_TRIGGER
#if TRIGGER_PULSE_WIDTH != 0
void UltrasonicAsyncPWM::_timer0() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[0], LOW);
	timer0.end();
}
void UltrasonicAsyncPWM::_timer1() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[1], LOW);
	timer1.end();
}
void UltrasonicAsyncPWM::_timer2() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[2], LOW);
	timer2.end();
}
void UltrasonicAsyncPWM::_timer3() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[3], LOW);
	timer3.end();
}
void UltrasonicAsyncPWM::_timer4() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[4], LOW);
	timer4.end();
}
void UltrasonicAsyncPWM::_timer5() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[5], LOW);
	timer5.end();
}
void UltrasonicAsyncPWM::_timer6() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[6], LOW);
	timer6.end();
}
void UltrasonicAsyncPWM::_timer7() {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();
	digitalWrite(_this->_trigger[7], LOW);
	timer7.end();
}
#endif
#endif

/** Add a sensor
@param trigger - Trigger pin.
@param echo - Pin for reading echno signal. Not necessary a PWM pin, but it must support interrupts.
*/
void UltrasonicAsyncPWM::add(uint8_t trigger, uint8_t echo) {
	if (nextFree >= MAX_ULTRASONIC_ASYNC_PWM)
		error("Too many sensors. Increase MAX_ULTRASONIC_ASYNC_PWM.");
	if (nextFree > 8)
		error("Not enough interrupt handlers.");

	_trigger[nextFree] = trigger;
	_echo[nextFree] = echo;
	_finished[nextFree] = false;
	pinMode(trigger, OUTPUT);
	digitalWrite(trigger, PULSE_IS_HIGH ? LOW : HIGH);
	pinMode(echo, INPUT);
	switch (nextFree) {
	case 0: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr0, CHANGE); break;
	case 1: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr1, CHANGE); break;
	case 2: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr2, CHANGE); break;
	case 3: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr3, CHANGE); break;
	case 4: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr4, CHANGE); break;
	case 5: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr5, CHANGE); break;
	case 6: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr6, CHANGE); break;
	case 7: attachInterrupt(digitalPinToInterrupt(echo), _echo_isr7, CHANGE); break;
	}
	nextFree++;
}

/** Distance in cm
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
@return - Distance.
*/
float UltrasonicAsyncPWM::distance(uint8_t sensorNumber) {
	return (_impulsEnd[sensorNumber] - _impulsStart[sensorNumber]) / 58.0;
}

/** Interrupt function
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
*/
void UltrasonicAsyncPWM::_echo_isr(uint8_t sensorNumber) {
	UltrasonicAsyncPWM* _this = UltrasonicAsyncPWM::instance();

	switch (digitalRead(_this->_echo[sensorNumber])) {
		case PULSE_IS_HIGH ? HIGH : LOW :
			_this->_impulsStart[sensorNumber] = micros();
			_this->_highDetected[sensorNumber] = true;
			break;
			case PULSE_IS_HIGH ? LOW : HIGH :
				if (_this->_highDetected[sensorNumber]) {
					_this->_impulsEnd[sensorNumber] = micros();
					_this->_finished[sensorNumber] = true;
				}
									   break;
	}
}

/** Print to all serial ports
@param message
@param eol - end of line
*/
void UltrasonicAsyncPWM::print(String message, bool eol) {
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

/** Trigger a pulse.
@param sensorNumber - Sensor's index. Function add() assigns 0 to first sensor, 1 to second, etc.
*/
void UltrasonicAsyncPWM::start(uint8_t sensorNumber) {
	_finished[sensorNumber] = false;
	_highDetected[sensorNumber] = false;
	digitalWrite(_trigger[sensorNumber], PULSE_IS_HIGH ? HIGH : LOW);
	bool isTimer = false;
#ifdef USE_TIMER_FOR_TRIGGER
#if TRIGGER_PULSE_WIDTH != 0
	switch (sensorNumber) {
	case 0: timer0.begin(_timer0, 10000); break;
	case 1: timer1.begin(_timer1, 10000); break;
	case 2: timer2.begin(_timer2, 10000); break;
	case 3: timer3.begin(_timer3, 10000); break;
	case 4: timer4.begin(_timer4, 10000); break;
	case 5: timer5.begin(_timer5, 10000); break;
	case 6: timer6.begin(_timer6, 10000); break;
	case 7: timer7.begin(_timer7, 10000); break;
	} //10 msec
	isTimer = true;
#endif
#endif
	if (!isTimer) {
		delayMicroseconds(TRIGGER_PULSE_WIDTH);
		digitalWrite(_trigger[sensorNumber], PULSE_IS_HIGH ? LOW : HIGH);
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void UltrasonicAsyncPWM::test(BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) {
			start(i); // Trigger a pulse
			uint32_t startMs = millis();
			bool ok = true;
			while (!isFinished(i)) // Wait for the result. If more than 100 ms, signal timeout.
				if (millis() - startMs > 100) {
					print("Timeout for sensor " + (String)i + " ");
					ok = false;
					break;
				}
			if (ok) // If ok, print the result
				print((String)distance(i) + "cm ");
		}
		print("", true);
		delay(200);
	}
}

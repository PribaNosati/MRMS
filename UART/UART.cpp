#include "UART.h"

/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - chosen UART. Example: UART(&Serial1);
@param speed - Sets the data rate in bits per second (baud) for serial data transmission. Use one of these rates: 300, 600, 1200, 2400, 4800,
9600, 14400, 19200, 28800, 38400, 57600, or 115200. The other party must use the same speed.
*/
UART::UART(HardwareSerial *hardwareSerial, uint32_t speed){
	serial = hardwareSerial;
	baud = speed;
}

UART::~UART(){
}

/** Starts serial communication
*/
void UART::add() {
	serial->begin(baud);
}

/**Get the number of bytes (characters) available for reading from the serial port.
@return - number of bytes.
*/
int8_t UART::available() {
	return serial->available();
}

/**Reads first byte of the incoming serial data.
@return - the first byte of incoming serial data available (or -1 if no data is available).
*/
int16_t UART::read()
{
	return serial->read();
}

/** Reads characters from the serial port into a buffer. The function terminates if the determined length has been read, or it times out.
@param buffer - the buffer to store the bytes in.
@param length - the number of bytes to read.
@return - the number of bytes placed in the buffer.
*/
uint8_t UART::read(uint8_t size, uint8_t * buffer)
{
	return serial->readBytes(buffer, size);
}

/** Writes a single byte to the serial port.
@param byte - a byte to send.
*/
void UART::write(uint8_t data)
{
	serial->write(data);
}

/** Writes a string to the serial port. To send the characters representing the digits of a	number use the print() function instead.
@param str - a string to send.
*/
void UART::write(const char *str)
{
	serial->write(str);
}

/** Writes series of bytes to the serial port; to send the characters representing the digits of a
number use the print() function instead.
@param size - buffer's size
@param buffer - data
*/
void UART::write(uint8_t size, uint8_t *buffer) {
	serial->write(buffer, size);
}
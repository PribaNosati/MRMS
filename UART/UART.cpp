#include "UART.h"

Message::Message(HardwareSerial *hardwareSerial) {
	bluetoothSerial = hardwareSerial;
}

/** A byte in messageText
@param index
*/
uint8_t& Message::operator[](uint8_t index) {
	return buffer[index];
}

/** Continue building messageText by appending to the tail
@param data - data to be appended
*/
void Message::append(uint8_t data) {
	if (nextBufferPos == MAXIMUM_MESSAGE_SIZE - 1)
		error("Message overflow");
	buffer[nextBufferPos] = data;
	nextBufferPos++;
	bufferTypes[nextTypesPos] = UINT8;
	nextTypesPos++;
}

/** Continue building messageText by appending to the tail
@param data - data to be appended
*/
void Message::append(uint16_t data) {
	if (nextBufferPos == MAXIMUM_MESSAGE_SIZE - 2)
		error("Message overflow");
	Mix mix;
	mix.int16 = data;
	buffer[nextBufferPos++] = mix.bytes[0];
	buffer[nextBufferPos++] = mix.bytes[1];
	bufferTypes[nextTypesPos] = UINT16;
	nextTypesPos++;
}

/** Continue building messageText by appending to the tail
@param data - data to be appended
*/
void Message::append(String data) {
	for (uint8_t i = 0; i < data.length(); i++)
		append((uint8_t)data[i]);
	append((uint8_t)0);
}

/** Buffer
@return - buffer
*/
uint8_t* Message::bytes() {
	return buffer;
}

/** Display content
*/
void Message::print() {
	print("message " + (String)size() + " bytes: ");
	for (uint8_t i = 0; i < size(); i++) {
		if (i != 0)
			print(", ");
		print((int)buffer[i]);
	}
}

/** Print to all serial ports
@param messageText
@param eol - end of line
*/
void Message::print(String messageText, bool eol) {
	if (eol) {
		Serial.println(messageText);
		if (bluetoothSerial != 0)
			bluetoothSerial->println(messageText);
	}
	else {
		Serial.print(messageText);
		if (bluetoothSerial != 0)
			bluetoothSerial->print(messageText);
	}
}

/** Read
@return - next
*/
uint8_t Message::readUInt8() {
	return buffer[nextReadPos++];
}

/** Read
@return - next
*/
uint16_t Message::readUInt16() {
	Mix mix;
	mix.bytes[0] = buffer[nextReadPos++];
	mix.bytes[1] = buffer[nextReadPos++];
	return mix.uint16;
}

/** Read
@return - next
*/
String Message::readString() {
	String str;
	while (uint8_t byte = buffer[nextReadPos++])
		str += byte;
	return str;
}

/** Clear messageText in order to start building a new one
*/
void Message::reset() {
	nextBufferPos = 0;
	nextTypesPos = 0;
}

/** Size
@return - number of bytes
*/
uint8_t Message::size() {
	return nextBufferPos;
}



/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - chosen UART. Example: UART(&Serial1);
@param speed - Sets the data rate in bits per second (baud) for serial data transmission. Use one of these rates: 300, 600, 1200, 2400, 4800,
9600, 14400, 19200, 28800, 38400, 57600, or 115200. The other party must use the same speed.
*/
UART::UART(HardwareSerial *hardwareSerial, uint32_t speed){
	uartSerial = hardwareSerial;
	baud = speed;
}

UART::~UART(){
}

/** Starts serial communication
*/
void UART::add() {
	uartSerial->begin(baud);
}

/**Get the number of bytes (characters) available for reading from the uartSerial port.
@return - number of bytes.
*/
int8_t UART::available() {
	return uartSerial->available();
}

/** Print to all serial ports
@param messageText
@param eol - end of line
*/
void UART::print(String message, bool eol) {
	if (eol) {
		Serial.println(message);
		if (bluetoothSerial != 0)
			bluetoothSerial->println(message);
	}
	else {
		Serial.print(message);
		if (bluetoothSerial != 0)
			bluetoothSerial->print(message);
	}
}

/**Reads first byte of the incoming serial data.
@return - the first byte of incoming serial data available (or -1 if no data is available).
*/
int16_t UART::read()
{
	return uartSerial->read();
}

/** Reads characters from the serial port into a buffer. The function terminates if the determined length has been read, or it times out.
@param buffer - the buffer to store the bytes in.
@param length - the number of bytes to read.
@return - the number of bytes placed in the buffer.
*/
uint8_t UART::read(uint8_t size, uint8_t * buffer)
{
	return uartSerial->readBytes(buffer, size);
}

/** Reads a messageText
@return - the messageText
@param verbose - print details
*/
Message UART::readMessage(bool verbose) {
	Message message;
	while (uartSerial->available())
		message.append((uint8_t)uartSerial->read());
	if (verbose) {
		print("Inbound ");
		message.print();
		print("", true);
	}
	return message;
}

/** Writes a single byte to the serial port.
@param byte - a byte to send.
*/
void UART::write(uint8_t data)
{
	uartSerial->write(data);
}

/** Writes a string to the serial port. To send the characters representing the digits of a	number use the print() function instead.
@param str - a string to send.
*/
void UART::write(const char *str)
{
	uartSerial->write(str);
}

/** Writes series of bytes to the serial port; to send the characters representing the digits of a
number use the print() function instead.
@param size - buffer's size
@param buffer - data
*/
void UART::write(uint8_t size, uint8_t *buffer) {
	uartSerial->write(buffer, size);
}

/** Writes a messageText to serial port.
@param messageText
@param verbose - print details
*/
void UART::write(Message message, bool verbose) {
	if (verbose) {
		print("Outbound ");
		message.print();
		print("", true);
	}
	write(message.size(), message.bytes());
}
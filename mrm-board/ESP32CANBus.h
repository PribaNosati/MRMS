#pragma once
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

#define rx_queue_size 10

class ESP32CANBus{
private:
	uint16_t bracketReceive[2] = { 0, 0 };
	uint16_t bracketSend[2] = { 0, 0 };
	uint8_t lastBracketReceive = 0;
	uint8_t lastBracketSend= 0;
	uint16_t _peakReceive = 0;
	uint16_t _peakSend = 0;

public:
	CAN_frame_t *rx_frame;
		
	ESP32CANBus();
	
	
	/**Receive a CANBus message
	@return - true if a message received
	*/
	bool messageReceive();
	
	/**Send a CANBus message
	@param stdId - CANBus message id
	@param dlc - data's used bytes count
	@param data - up to 8 data bytes
	@return - true if a message received
	*/
	void messageSend(uint32_t stdId, uint8_t dlc, uint8_t data[8]);

	/** Number of received CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesReceivedPerSecond() { return (lastBracketReceive == 1 ? bracketReceive[0] : bracketReceive[1]) * 2; }

	/** Number of sent CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesSentPerSecond() { return (lastBracketSend == 1 ? bracketSend[0] : bracketSend[1]) * 2; }

	/** Peak number of received CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesPeakReceived() { return _peakReceive * 2; }

	/** Peak number of received CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesPeakSent() { return _peakSend * 2; }

};
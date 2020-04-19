#pragma once
#include <Arduino.h>

#define QUEUE_LENGTH 20

struct CANBusMessage* dequeBack();
bool dequeEmpty();
void dequeInitialize();
void dequePopBack();
void messageReceiveAsync(int packetSize);

/** Push a message. queueFirst points to the oldest message and queueLast to the latest.
 */
bool dequePushFront(uint32_t msgId, uint8_t dlc, uint8_t data[8]);

struct CANBusMessage {
	uint32_t messageId;
	uint8_t dlc;
	uint8_t data[8];

	void print();
};

class Mrm_can_bus{
private:
public:
		
	Mrm_can_bus();
	
	/**Receive a CANBus message
	@return true - a message received, false - none
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
	uint16_t messagesReceivedPerSecond();

	/** Number of sent CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesSentPerSecond();

	/** Peak number of received CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesPeakReceived();

	/** Peak number of received CAN Bus messages per second
	@return - number of messages
	*/
	uint16_t messagesPeakSent();

	void messagesReset();

};
#include <mrm-can-bus.h>
#include <mrm-can-esp32.h>
#include <mrm-common.h>

/**
Purpose: common library for ESP32 CANBus access
@author MRMS team
@version 0.0 2020-04-18
Licence: You can use this code any way you like.
*/

#define VERBOSE 0
#define CAN_RECEIVE_INTERRUPT 1

struct CANBusMessage* queue[QUEUE_LENGTH];
uint8_t queueNextBack = 0;

uint16_t bracketReceive[2] = { 0, 0 };
uint16_t bracketSend[2] = { 0, 0 };
uint8_t lastBracketReceive = 0;
uint8_t lastBracketSend = 0;
uint16_t _peakReceive = 0;
uint16_t _peakSend = 0;

void CANBusMessage::print() {
	::print("Id: 0x%04X, data:", messageId);
	for (uint8_t i = 0; i < dlc; i++)
		::print(" %02X", data[i]);
	::print("\n\r");
}

struct CANBusMessage* dequeBack() {
	if (queueNextBack == 0)
		return NULL;
	else
		return queue[queueNextBack - 1];
}

bool dequeEmpty() { return queueNextBack == 0; }

void dequeInitialize() {
	for (uint8_t i = 0; i < QUEUE_LENGTH; i++) {
		struct CANBusMessage* p = (CANBusMessage*)malloc(sizeof(struct CANBusMessage));
		queue[i] = p;
	}
}

void dequePopBack() {
	if (queueNextBack > 0)
		queueNextBack--;
}

/** Push a message. queueFirst points to the oldest message and queueLast to the latest.
 */
bool dequePushFront(uint32_t msgId, uint8_t dlc, uint8_t data[8]) {
	bool ok = true;
	if (queueNextBack >= QUEUE_LENGTH) {
		dequePopBack();
		ok = false;
	}
	struct CANBusMessage* ptr = queue[QUEUE_LENGTH - 1];
	for (uint8_t i = QUEUE_LENGTH - 1; i > 0; i--)
		queue[i] = queue[i - 1];
	ptr->messageId = msgId;
	ptr->dlc = dlc;
	for (uint8_t i = 0; i < 8; i++)
		ptr->data[i] = data[i];
	queue[0] = ptr;

	queueNextBack++;
	//print("Q:%i\n\r", queueNextBack);
	return ok;
}


Mrm_can_bus::Mrm_can_bus(){
	  // start the CAN bus at 250 kbps
	dequeInitialize();
	if (!CAN.begin(250E3)) {
		strcpy(errorMessage, "Starting CAN failed");
		return;
	}
#if CAN_RECEIVE_INTERRUPT
	CAN.onReceive(messageReceiveAsync);
#endif
}

/**Receive a CANBus message
@return true - a message received, false - none
*/
bool Mrm_can_bus::messageReceive() {
#if CAN_RECEIVE_INTERRUPT
	return false;
#endif
	bool found = false;
	int packetSize = 0;
#define CAN_QUEUE 1
#if CAN_QUEUE
	do {
		packetSize = CAN.parsePacket();
		if (packetSize) {
			uint8_t i = 0;
			uint8_t data[8];
			while (CAN.available())
				data[i++] = CAN.read();
			if (!dequePushFront(CAN.packetId(), packetSize, data)) {
				strcpy(errorMessage, "Deque full.");
				return false;
			}

			uint16_t bracketNow = millis() % 1000;
			if (bracketNow >= 500) {
				if (lastBracketReceive == 0) {
					lastBracketReceive = 1;
					if (bracketReceive[0] > _peakReceive)
						_peakReceive = bracketReceive[0];
					bracketReceive[1] = 0;
				}
			}
			else {
				if (lastBracketReceive == 1) {
					lastBracketReceive = 0;
					if (bracketReceive[1] > _peakReceive)
						_peakReceive = bracketReceive[1];
					bracketReceive[0] = 0;
				}
			}
			bracketReceive[lastBracketReceive]++;
		}
	} while (packetSize);
#else
	rx_frame->MsgID = CAN.packetId();
	rx_frame->FIR.B.DLC = packetSize;

	uint8_t i = 0;
	while (CAN.available())
		rx_frame->data.u8[i++] = CAN.read();
	found = true;

	uint16_t bracketNow = millis() % 1000;
	if (bracketNow >= 500) {
		if (lastBracketReceive == 0) {
			lastBracketReceive = 1;
			if (bracketReceive[0] > _peakReceive)
				_peakReceive = bracketReceive[0];
			bracketReceive[1] = 0;
		}
	}
	else {
		if (lastBracketReceive == 1) {
			lastBracketReceive = 0;
			if (bracketReceive[1] > _peakReceive)
				_peakReceive = bracketReceive[1];
			bracketReceive[0] = 0;
		}
	}
	bracketReceive[lastBracketReceive]++;
#endif

//
//	// Receive next CAN frame from queue
//	if (xQueueReceive(CAN_cfg.rx_queue, rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
//#if VERBOSE
//		if (rx_frame->FIR.B.FF == CAN_frame_std) {
//			printf("Rcvd. frame");
//		}
//		else {
//			printf("Rcvd. ext. frame");
//		}
//
//		if (rx_frame->FIR.B.RTR == CAN_RTR) {
//			printf(" RTR from 0x%04X, DLC %d\r\n", rx_frame->MsgID, rx_frame->FIR.B.DLC);
//		}
//		else {
//			printf(" from 0x%04X, DLC %d, Data ", rx_frame->MsgID, rx_frame->FIR.B.DLC);
//			for (int i = 0; i < rx_frame->FIR.B.DLC; i++) {
//				printf("0x%02X ", rx_frame->data.u8[i]);
//			}
//			printf("\n\r");
//		}
//#endif

	return found;
}

void messageReceiveAsync(int packetSize) {
	uint8_t i = 0;
	uint8_t data[8];
	while (CAN.available()) {
		if (i >= 8) {
			print("Overflow\n\r");
			return;
		}
		data[i++] = CAN.read();
	}
	if (!dequePushFront(CAN.packetId(), packetSize, data)) {
		strcpy(errorMessage, "Deque full");
		return;
	}

	uint16_t bracketNow = millis() % 1000;
	if (bracketNow >= 500) {
		if (lastBracketReceive == 0) {
			lastBracketReceive = 1;
			if (bracketReceive[0] > _peakReceive)
				_peakReceive = bracketReceive[0];
			bracketReceive[1] = 0;
		}
	}
	else {
		if (lastBracketReceive == 1) {
			lastBracketReceive = 0;
			if (bracketReceive[1] > _peakReceive)
				_peakReceive = bracketReceive[1];
			bracketReceive[0] = 0;
		}
	}
	bracketReceive[lastBracketReceive]++;
}

/** Number of received CAN Bus messages per second
@return - number of messages
*/
uint16_t Mrm_can_bus::messagesReceivedPerSecond() { return (lastBracketReceive == 1 ? bracketReceive[0] : bracketReceive[1]) * 2; }

/** Number of sent CAN Bus messages per second
@return - number of messages
*/
uint16_t Mrm_can_bus::messagesSentPerSecond() { return (lastBracketSend == 1 ? bracketSend[0] : bracketSend[1]) * 2; }

/** Peak number of received CAN Bus messages per second
@return - number of messages
*/
uint16_t Mrm_can_bus::messagesPeakReceived() { return _peakReceive * 2; }

/** Peak number of received CAN Bus messages per second
@return - number of messages
*/
uint16_t Mrm_can_bus::messagesPeakSent() { return _peakSend * 2; }

void Mrm_can_bus::messagesReset() {
	for (uint8_t i = 0; i < 2; i++) {
		bracketReceive[i] = 0;
		bracketSend[i] = 0;
	}
	_peakReceive = 0;
	_peakSend = 0;
}

/**Send a CANBus message
@param stdId - CANBus message id
@param dlc - data's used bytes count
@param data - up to 8 data bytes
@return - true if a message received
*/
void Mrm_can_bus::messageSend(uint32_t stdId, uint8_t dlc, uint8_t data[8]) {
	static uint32_t lastSentMs = 0;
#define DELAY_MESSAGES 0
#if DELAY_MESSAGES
	if (millis() == lastSentMs)
		delay(1);
#endif
	if (!CAN.beginPacket(stdId, dlc)) {
		strcpy(errorMessage, "Error sending");
		return;
	}
	if (CAN.write(data, dlc) != dlc) {
		strcpy(errorMessage, "Wrong byte count");
		return;
	}
	CAN.endPacket();

	lastSentMs = millis();
#if VERBOSE
	printf("Send to 0x%04X, DLC %d, Data ", stdId, dlc);
	for (int i = 0; i < dlc; i++) {
		printf("0x%02X ", data[i]);
	}
	printf("\n\r");
#endif

	uint16_t bracketNow = millis() % 1000;
	if (bracketNow >= 500) {
		if (lastBracketSend == 0) {
			lastBracketSend = 1;
			if (bracketSend[0] > _peakSend)
				_peakSend = bracketSend[0];
			bracketSend[1] = 0;
		}
	}
	else {
		if (lastBracketSend == 1) {
			lastBracketSend = 0;
			if (bracketSend[1] > _peakSend)
				_peakSend = bracketSend[1];
			bracketSend[0] = 0;
		}
	}
	bracketSend[lastBracketSend]++;
}
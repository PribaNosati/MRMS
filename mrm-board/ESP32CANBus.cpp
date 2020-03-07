#include "ESP32CANBus.h"
#include <ESP32CAN.h>

/**
Purpose: common library for ESP32 CANBus access
@author MRMS team
@version 0.0 2019-07-20
Licence: You can use this code any way you like.
*/

#define VERBOSE 0

CAN_device_t CAN_cfg;  

ESP32CANBus::ESP32CANBus(){
	rx_frame = new CAN_frame_t();
	CAN_cfg.speed = CAN_SPEED_250KBPS;
	CAN_cfg.tx_pin_id = GPIO_NUM_5;
	CAN_cfg.rx_pin_id = GPIO_NUM_4;
	CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
	// Init CAN Module
	ESP32Can.CANInit();
}

/**Receive a CANBus message
@return - true if a message received
*/
bool ESP32CANBus::messageReceive() {

	// Receive next CAN frame from queue
	if (xQueueReceive(CAN_cfg.rx_queue, rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
#if VERBOSE
		if (rx_frame->FIR.B.FF == CAN_frame_std) {
			printf("Rcvd. frame");
		}
		else {
			printf("Rcvd. ext. frame");
		}

		if (rx_frame->FIR.B.RTR == CAN_RTR) {
			printf(" RTR from 0x%04X, DLC %d\r\n", rx_frame->MsgID, rx_frame->FIR.B.DLC);
		}
		else {
			printf(" from 0x%04X, DLC %d, Data ", rx_frame->MsgID, rx_frame->FIR.B.DLC);
			for (int i = 0; i < rx_frame->FIR.B.DLC; i++) {
				printf("0x%02X ", rx_frame->data.u8[i]);
			}
			printf("\n\r");
		}
#endif
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

		return true;
	}
	else 
		return false;
}

/**Send a CANBus message
@param stdId - CANBus message id
@param dlc - data's used bytes count
@param data - up to 8 data bytes
@return - true if a message received
*/
void ESP32CANBus::messageSend(uint32_t stdId, uint8_t dlc, uint8_t data[8]) {
	static uint32_t lastSentMs = 0;
	if (millis() == lastSentMs)
		delay(1);
	static CAN_frame_t tx_frame;
	tx_frame.FIR.B.FF = CAN_frame_std;
	tx_frame.MsgID = stdId;
	tx_frame.FIR.B.DLC = dlc;
	for (uint8_t i = 0; i < 8; i++)
		tx_frame.data.u8[i] = data[i];
	ESP32Can.CANWriteFrame(&tx_frame);
	lastSentMs = millis();
#if VERBOSE
	printf("Send to 0x%04X, DLC %d, Data ", tx_frame.MsgID, tx_frame.FIR.B.DLC);
	for (int i = 0; i < tx_frame.FIR.B.DLC; i++) {
		printf("0x%02X ", tx_frame.data.u8[i]);
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
	bracketSend[lastBracketReceive]++;
}
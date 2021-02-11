#include "mrm-can-bus.h"
#include <mrm-common.h>

#include <driver/can.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/**
Purpose: common library for ESP32 CANBus access
@author MRMS team
@version 0.0 2020-05-31
Licence: You can use this code any way you like.
*/

#define VERBOSE 0

uint16_t bracketReceive[2] = { 0, 0 };
uint16_t bracketSend[2] = { 0, 0 };
uint8_t lastBracketReceive = 0;
uint8_t lastBracketSend = 0;
uint16_t _peakReceive = 0;
uint16_t _peakSend = 0;

CANBusMessage* receivedMessage = NULL;

void CANBusMessage::print() {
	::print("Id: 0x%04X, data:", messageId);
	for (uint8_t i = 0; i < dlc; i++)
		::print(" %02X", data[i]);
	::print("\n\r");
}

Mrm_can_bus::Mrm_can_bus() {
	can_general_config_t general_config = {
	   .mode = CAN_MODE_NORMAL,
	   .tx_io = (gpio_num_t)GPIO_NUM_5,
	   .rx_io = (gpio_num_t)GPIO_NUM_4,
	   .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
	   .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
	   .tx_queue_len = 20,
	   .rx_queue_len = 65,
	   .alerts_enabled = CAN_ALERT_NONE,
	   .clkout_divider = 0 };
	can_timing_config_t timing_config = CAN_TIMING_CONFIG_250KBITS();
	can_filter_config_t filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

	if (can_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK)
		strcpy(errorMessage, "Error init. CAN");

	if (can_start() != ESP_OK)
		strcpy(errorMessage, "Error start CAN");

	receivedMessage = new CANBusMessage();
}

/**Receive a CANBus message
@return non-NULL - a message received, NULL - none
*/
CANBusMessage* Mrm_can_bus::messageReceive() {
	//Wait for message to be received
	bool found = false;
	can_message_t message;
	esp_err_t status = can_receive(&message, pdMS_TO_TICKS(1)); // When 0, lost messages
	switch(status){
	case ESP_OK:
		found = true;
		break;
	case ESP_ERR_TIMEOUT:
		found = false;
		break;
	default:
		strcpy(errorMessage, "Error receiving");
		return NULL;
	}

	if (found) {

		for (uint8_t i = 0; i < message.data_length_code; i++)
			receivedMessage->data[i] = message.data[i];
		receivedMessage->messageId = message.identifier;
		receivedMessage->dlc = message.data_length_code;


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

		return receivedMessage;
	}
	else
		return NULL;
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
	can_message_t message;
	message.identifier = stdId;
	message.flags = 0;
	message.data_length_code = dlc;
	for (int i = 0; i < dlc; i++) {
		message.data[i] = data[i];
	}

	// Do not allow bus congestion, limit to 1250 messages per second. After 70 min. micros() resets, therefore the second term in the logical expression below
#define MIN_MICROS_BETWEEN_CAN_BUS_MESSAGES 900 //900
	while (micros() <= lastSentMicros + MIN_MICROS_BETWEEN_CAN_BUS_MESSAGES){
		 if (micros() < lastSentMicros){ // micros() resetted
			lastSentMicros = 0; // reset this variable, too
		 }
	}
	lastSentMicros = micros();

	//Queue message for transmission
	if (can_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK)
		strcpy(errorMessage, "Error sending");

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
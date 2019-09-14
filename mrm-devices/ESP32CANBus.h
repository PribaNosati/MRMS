#pragma once
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

#define rx_queue_size 10

class ESP32CANBus{
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
};
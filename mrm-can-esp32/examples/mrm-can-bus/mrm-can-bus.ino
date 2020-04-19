#include <mrm-can-bus.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes

void setup() {
  Serial.begin(115200);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x182 + 2 * i, 1, data);
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x280 + 2 * i, 1, data);
}

void loop() {
  // Receive a message
	can.messageReceive();
	while (!dequeEmpty()) {
		CANBusMessage *msg = dequeBack();
		dequePopBack();
		msg->print();
	}
}

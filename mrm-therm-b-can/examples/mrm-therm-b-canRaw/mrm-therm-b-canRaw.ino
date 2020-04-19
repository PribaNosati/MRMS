#include <mrm-can-bus.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_SENDING 0x13

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes

void setup() {
  Serial.begin(115200);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x210 + 2 * i, 1, data); //0x210, 0x212, ... , 0x21E.
}

void loop() {
  // Receive a message
  while (!dequeEmpty()) {
	CANBusMessage *msg = dequeBack();
	dequePopBack();
	if (msg->data[0] ==  COMMAND_SENSORS_MEASURE_SENDING){
          uint16_t deg = (msg->data[2] << 8) | msg->data[1];
          Serial.print(deg);
          Serial.println(" deg C");
    }
  }
}

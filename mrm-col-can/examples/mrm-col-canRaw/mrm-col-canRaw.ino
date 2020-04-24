#include <mrm-can-bus.h>

#define COMMAND_SENDING_COLORS_1_TO_3 0x06
#define COMMAND_SENDING_COLORS_4_TO_6 0x07
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes
uint16_t readings[6];

void setup() {
  Serial.begin(115200);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x310 + 2 * i, 1, data);
}

void loop() {
  // Receive a message
  uint8_t startIndex;
  while (!dequeEmpty()) {
  CANBusMessage *msg = dequeBack();
  dequePopBack();
    switch (msg->data[0]) {
    case COMMAND_SENDING_COLORS_1_TO_3:
      readings[0] = (msg->data[1] << 8) | msg->data[2]; // blue
      readings[1] = (msg->data[3] << 8) | msg->data[4]; // green
      readings[2] = (msg->data[5] << 8) | msg->data[6]; // orange
      break;
    case COMMAND_SENDING_COLORS_4_TO_6:
      readings[3] = (msg->data[1] << 8) | msg->data[2]; // red
      readings[4] = (msg->data[3] << 8) | msg->data[4]; // violet
      readings[5] = (msg->data[5] << 8) | msg->data[6]; // yellow
      break;
    }
  }

  // Display results
  static uint32_t ms = 0;
  if (millis() - ms > 300){
      printf("Blu:%3i Grn:%3i Ora:%3i Red:%3i Vio:%3i Ylw:%3i\n\r", readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);
    ms = millis();
  }
}

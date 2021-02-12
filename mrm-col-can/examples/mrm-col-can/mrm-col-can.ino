#include <mrm-can-bus.h>

// A single CAN Bus message cannot hold all the colors. Therefore, 2 of them are needed.
#define COMMAND_SENDING_COLORS_1_TO_3 0x06 // Command indicating payload: first 3 colors
#define COMMAND_SENDING_COLORS_4_TO_6 0x07 // Command indicating payload: last 3 colors
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10 // Command to start measuring.

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes
uint16_t readings[6];   // Array to hold 6 colors

void setup() {
  Serial.begin(115200); // Adjust monitor speed to 115200 bps in order not to get garbage in the window.

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content will hold the command that will order the sensor to start sending colors
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x310 + 2 * i, 1, data); // 0x310 is sensor 1, 0x312 sensor 2, etc. This loop will start all the sensors that are connected to the bus.
}

void loop() {
  // Receive a message
  uint8_t startIndex;
  CANBusMessage* msg = can.messageReceive(); // Receive a message, if any arrived.
  if (msg != NULL){ // If not NULL, a message received.
    switch (msg->data[0]) {// According to command (header) decode the appropriate payload
    case COMMAND_SENDING_COLORS_1_TO_3: // First 3 colors
      readings[0] = (msg->data[1] << 8) | msg->data[2]; // blue
      readings[1] = (msg->data[3] << 8) | msg->data[4]; // green
      readings[2] = (msg->data[5] << 8) | msg->data[6]; // orange
      break;
    case COMMAND_SENDING_COLORS_4_TO_6: // Last 3 colors.
      readings[3] = (msg->data[1] << 8) | msg->data[2]; // red
      readings[4] = (msg->data[3] << 8) | msg->data[4]; // violet
      readings[5] = (msg->data[5] << 8) | msg->data[6]; // yellow
      break;
    }
  }

  // Display results
  static uint32_t ms = 0;
  if (millis() - ms > 300){ // If 300 ms since the last display, show again.
      printf("Blu:%3i Grn:%3i Ora:%3i Red:%3i Vio:%3i Ylw:%3i\n\r", readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);
    ms = millis();
  }
}

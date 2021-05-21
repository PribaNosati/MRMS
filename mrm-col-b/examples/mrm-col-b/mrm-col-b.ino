#include <mrm-can-bus.h>

// A single CAN Bus message cannot hold all the colors. Therefore, 2 of them are needed.
#define COMMAND_SENDING_COLORS_1_TO_3 0x06        // Command indicating payload: first 3 colors.
#define COMMAND_SENDING_COLORS_4_TO_6 0x07        // Command indicating payload: second 3 colors.
#define COMMAND_SENDING_COLORS_7_TO_9 0x08        // Command indicating payload: third 3 colors.
#define COMMAND_SENDING_COLORS_10_TO_11 0x09      // Command indicating payload: last 2 colors.
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10   // CAN Bus command that instructs the sensor to start sending colors.

Mrm_can_bus can;

uint8_t data[8];                                  // Message content: 8 bytes
uint16_t readings[10];                            // Array to hold 10 colors. The last channel is flicker and is not in the payload.

void setup() {
  Serial.begin(115200);                           // Adjust monitor speed to 115200 bps in order not to get garbage in the window.

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;   // First byte of the content will hold the command that will order the sensor to start sending colors
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x380 + 2 * i, 1, data);      // 0x380 is sensor 1, 0x382 sensor 2, etc. This loop will start all the sensors that are connected to the bus.
}

void loop() {
  // Receive a message
  uint8_t startIndex;
  CANBusMessage* msg = can.messageReceive();            // Receive a message, if any arrived.
  if (msg != NULL){                                     // If not NULL, a message received.
    switch (msg->data[0]) {                             // According to command (header) decode the appropriate payload
    case COMMAND_SENDING_COLORS_1_TO_3:                 // First 3 colors
      readings[0] = (msg->data[1] << 8) | msg->data[2]; // violet
      readings[1] = (msg->data[3] << 8) | msg->data[4]; // blue violetish
      readings[2] = (msg->data[5] << 8) | msg->data[6]; // blue
      break;
    case COMMAND_SENDING_COLORS_4_TO_6:                 // Last 3 colors.
      readings[3] = (msg->data[1] << 8) | msg->data[2]; // blue greenish
      readings[4] = (msg->data[3] << 8) | msg->data[4]; // green
      readings[5] = (msg->data[5] << 8) | msg->data[6]; // yellow
      break;
    case COMMAND_SENDING_COLORS_7_TO_9:                 // Last 3 colors.
      readings[6] = (msg->data[1] << 8) | msg->data[2]; // orange
      readings[7] = (msg->data[3] << 8) | msg->data[4]; // red
      readings[8] = (msg->data[5] << 8) | msg->data[6]; // near IR
      break;
    case COMMAND_SENDING_COLORS_10_TO_11:               // Last 3 colors.
      readings[9] = (msg->data[1] << 8) | msg->data[2]; // clear (white)
      break;
    }
  }

  // Display results
  static uint32_t ms = 0;
  if (millis() - ms > 300){ // If 300 ms since the last display, show again.
      printf("Vi:%3i B1:%3i B2:%3i B3:%3i Gr:%3i Ye:%3i Or:%3i Re:%3i IR:%3i Wh:%3i\n\r", readings[0], readings[1], readings[2], readings[3], 
        readings[4], readings[5], readings[6], readings[7], readings[8], readings[9], readings[10]);
    ms = millis();
  }
}

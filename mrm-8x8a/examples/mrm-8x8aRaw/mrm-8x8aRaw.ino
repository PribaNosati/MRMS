#include <mrm-can-bus.h>

#define COMMAND_SWITCH_ON 0x01
#define COMMAND_8X8_BITMAP_DISPLAY_PART1 0x05
#define COMMAND_8X8_BITMAP_DISPLAY_PART2 0x06
#define COMMAND_8X8_BITMAP_DISPLAY_PART3 0x07
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_SENDING 0x13

Mrm_can_bus can;
uint8_t data[8];        // Message content: 8 bytes

void setup() {
  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  can.messageSend(0x200, 1, data);
}

void loop() {
  // Receive a message
  CANBusMessage* msg = can.messageReceive();
  if (msg != NULL && msg->data[0] == COMMAND_SWITCH_ON){

    uint8_t red[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000100, 0b00111000, 0b00000000, 0b00111100 };
    uint8_t green[8] = { 0b00111100, 0b01000010, 0b10101001, 0b10101001, 0b10000001, 0b10000001, 0b01000010, 0b00111100 };
    
    data[0] = COMMAND_8X8_BITMAP_DISPLAY_PART1;
    for (uint8_t i = 0; i < 7; i++) 
      data[i + 1] = green[i];
    can.messageSend(0x200, 8, data);

    data[0] = COMMAND_8X8_BITMAP_DISPLAY_PART2;
    data[1] = green[7];
    for (uint8_t i = 0; i < 6; i++) 
      data[i + 2] = red[i];
    can.messageSend(0x200, 8, data);

    data[0] = COMMAND_8X8_BITMAP_DISPLAY_PART3;
    for (uint8_t i = 0; i < 2; i++) 
      data[i + 1] = red[i + 6];
    can.messageSend(0x200, 3, data);
  }
}

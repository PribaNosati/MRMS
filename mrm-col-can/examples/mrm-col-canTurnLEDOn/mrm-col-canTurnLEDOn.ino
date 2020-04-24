#include <mrm-can-bus.h>

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes
uint16_t readings[6];

void setup() {
  Serial.begin(115200);

  data[0] = 0x50;  // First byte of the content
  data[1] = 2;
   for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x310 + 2 * i, 2, data);
    Serial.print("Start");
}

void loop() {
}

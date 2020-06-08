#include <mrm-can-bus.h>

Mrm_can_bus can;
uint32_t ms = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {

  // Send a message
  if (millis() - ms > 1000){
    uint32_t stdId = 0x180; // Message id
    uint8_t dlc = 1;        // Message byte count
    uint8_t data[8];        // Message content: 8 bytes
    data[0] = 0xFF;         // First byte of the content
    can.messageSend(stdId, dlc, data);
    printf("Message sent\n\r");
    ms = millis();
  }

  // Receive a message
  if (can.messageReceive()){
    printf("Message received\n\r");
  }
}

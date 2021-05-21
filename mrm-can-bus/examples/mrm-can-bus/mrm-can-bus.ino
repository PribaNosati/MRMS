#include <mrm-can-bus.h>                // Include the library.

Mrm_can_bus can;                        // CAN Bus object.
uint32_t ms = 0;

void setup() {
  Serial.begin(115200);                 // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
}

void loop() {

  // Send a message
  if (millis() - ms > 1000){            // Every second...
    uint32_t stdId = 0x180;             // Message id
    uint8_t dlc = 1;                    // Message byte count
    uint8_t data[8];                    // Message content: 8 bytes
    data[0] = 0xFF;                     // First byte of the content
    can.messageSend(stdId, dlc, data);  // Send the message
    printf("Message sent\n\r");
    ms = millis();
  }

  // Receive a message
  if (can.messageReceive()){
    printf("Message received\n\r");
  }
}

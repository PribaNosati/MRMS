#include <mrm-can-bus.h>

#define CAN_COL_ILLUMINATION_CURRENT 0x50 // Command that sets sensor's illumination LED current.

Mrm_can_bus can;  // CAN Bus object.

uint8_t data[8];        // Message content: 8 bytes

void setup() {
  Serial.begin(115200); // Adjust monitor speed to 115200 bps in order not to get garbage in the window.

  data[0] = CAN_COL_ILLUMINATION_CURRENT;  // First byte of the content is the command.
  data[1] = 2; // Second byte is the current, values between 0 and 3. 0 switches the LED off.
  for (uint8_t i = 0; i < 8; i++) // For all the sensors...
    can.messageSend(0x310 + 2 * i, 2, data); // 0x310 is sensor 1, 0x312 sensor 2, etc. This loop will choolse all the sensors that are connected to the bus and instruct them all to switch their LEDs on.
  Serial.print("Start");
}

void loop() {
}

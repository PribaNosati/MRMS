#include <mrm-can-bus.h>

#define ID_FET 0x0340                   // Address of the switch.

#define COMMAND_TURN_ON 0x50            // Command to turn the switch on.
#define COMMAND_TURN_OFF 0x51           // Command to turn the switch off.

Mrm_can_bus can;                        // CAN Bus object.
uint8_t data[8];                        // Message content: 8 bytes

void setup() {
  Serial.begin(115200);                 // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
}

void loop() {
  static uint32_t ms = 0;
  static bool on = false;
  if (millis() - ms > 1000){            // Every second...
    on = !on;                           // OFF -> ON or ON -> OFF.
    Serial.println(on ? "On" : "Off");
    data[0] = on ? COMMAND_TURN_OFF : COMMAND_TURN_ON;
    data[1] = 0;                        // Output number (0 or 1).
    can.messageSend(ID_FET, 2, data);   // This command will open or close FET output (switch).
    ms = millis();
  }
}

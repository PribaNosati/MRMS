#include <mrm-can-bus.h>

#define COMMAND_IR_FINDER3_SENDING_SENSORS_1_TO_7 0x04  // Command indicating that the sensor sends values of receivers 1 to 7
#define COMMAND_IR_FINDER3_SENDING_SENSORS_8_TO_12 0x05 // Command indicating that the sensor sends values of receivers 8 to 12
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10         // Command that instruct the sensor to start sending readings.

Mrm_can_bus can;

uint8_t data[8];                                        // Message content: 8 bytes.
uint16_t reading[12];                                   // Array to hold all the readings (12 receivers).
bool near;                                              // Ball is near.

void setup() {
  Serial.begin(115200);                                 // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
  delay(1000);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;         // First byte of the content. We will order the sensor to start sending readings.
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x330 + 2 * i, 1, data);            // 0x330 is sensor 1, 0x332 sensor 2, etc. This loop will start all the sensors that are connected to the bus.
}

void loop() {
  uint8_t startIndex = 0;

  CANBusMessage* msg = can.messageReceive();            // Receive a message, if any arrived.
  if (msg != NULL){                                     // If not NULL, a message received.
    uint8_t length = 7;
    switch (msg->data[0]) {                             // Choose type of payload, for the receivers 1 to 7 or 8 to 12.
    case COMMAND_IR_FINDER3_SENDING_SENSORS_1_TO_7:
                                                        // "startIndex" is already 0 and "length" is already 7, so nothing to do here.
      break;
    case COMMAND_IR_FINDER3_SENDING_SENSORS_8_TO_12:
      startIndex = 7;                                   // Set startIndex to fill correct values of variable "reading" later in code.
      length = 5;                                       // This message carries only 5 bytes of payload, instead of 7, as in the previous message.
      near = msg->data[6];                              // One of the remaining bytes is use to carry information if the ball is near.
      break;
    default:
      startIndex = 0xFF;                                // Some other CAN Bus command, do nothing later.
    }

    for (uint8_t i = 0; i < length && startIndex != 0xFF; i++) // Rad the values from the message's payload.
      reading[startIndex + i] = msg->data[i + 1];
  }

  // Display results
  static uint32_t ms = 0;
  if (millis() - ms > 500 && startIndex == 7){
    for (uint8_t i = 0; i < 12; i++)
      Serial.print(reading[i]), Serial.print(" ");
    Serial.println(near ? " near" : " far");
    ms = millis();
  }
}

#include <mrm-can-bus.h>

// A single CAN Bus message cannot hold all the readings. Therefore, 3 of them are needed.
#define COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3 0x06     // Command indicating payload: first 3 transistors
#define COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6 0x07     // Command indicating payload: second 3 transistors
#define COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9 0x08     // Command indicating payload: third 3 transistors
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10         // Command to start measuring.

Mrm_can_bus can;

uint8_t data[8];                                        // Message content: 8 bytes
uint16_t reading[9];                                    // Array to hold 9 readings

void setup() {
  Serial.begin(115200);                                 // Adjust monitor speed to 115200 bps in order not to get garbage in the window.

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;         // First byte of the content will hold the command that will order the sensor to start sending readings
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x160 + 2 * i, 1, data);            // 0x160 is sensor 1, 0x162 sensor 2, etc. This loop will start all the sensors that are connected to the bus.
}

void loop() {
  // Receive a message
  uint8_t startIndex;
  CANBusMessage* msg = can.messageReceive();      // Receive a message, if any arrived.
  if (msg != NULL){                               // If not NULL, a message received.
    switch (msg->data[0]) {// According to command (header) decode the appropriate payload
    case COMMAND_REF_CAN_SENDING_SENSORS_1_TO_3:  // First 3 transistors
      startIndex = 0;                             // It will be used to populate the right part of the "rading" array.
      break;
    case COMMAND_REF_CAN_SENDING_SENSORS_4_TO_6:  // Second 3 transistors
      startIndex = 3;
      break;
    case COMMAND_REF_CAN_SENDING_SENSORS_7_TO_9:  // Third 3 transistors
      startIndex = 6;
      break;
    default:                                      // If some other CAN Bus message, do nothing
      startIndex = 0xFF;
    }
    
    for (uint8_t i = 0; i <= 2 && startIndex != 0xFF; i++)
      reading[startIndex + i] = (msg->data[2 * i + 1] << 8) | msg->data[2 * i + 2]; // Decode readings from 2 bytes.
  }

  // Display results
  static uint32_t ms = 0;
  if (millis() - ms > 500){
    for (uint8_t i = 0; i < 9; i++)
      Serial.print(reading[i]), Serial.print(" ");
    Serial.println();
    ms = millis();
  }
}

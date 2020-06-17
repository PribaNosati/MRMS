#include <mrm-can-bus.h>

#define COMMAND_IR_FINDER3_SENDING_SENSORS_1_TO_7 0x04
#define COMMAND_IR_FINDER3_SENDING_SENSORS_8_TO_12 0x05
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_IR_FINDER3_SENDING_ANGLE_AND_DISTANCE 0x09

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes
uint16_t reading[12];
bool near;

void setup() {
  Serial.begin(115200);
  delay(1000);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x330 + 2 * i, 1, data);
}

void loop() {
  uint8_t startIndex = 0;
  // Receive a message
  CANBusMessage* msg = can.messageReceive();
  if (msg != NULL){
    uint8_t length = 7;
    switch (msg->data[0]) {
    case COMMAND_IR_FINDER3_SENDING_SENSORS_1_TO_7:
      break;
    case COMMAND_IR_FINDER3_SENDING_SENSORS_8_TO_12:
      startIndex = 7;
      length = 5;
      near = msg->data[6];
      break;
    default:
      startIndex = 0xFF;
    }
    
    for (uint8_t i = 0; i < length && startIndex != 0xFF; i++)
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

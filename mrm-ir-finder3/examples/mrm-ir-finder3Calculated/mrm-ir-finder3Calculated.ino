#include <mrm-can-bus.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA 0x15
#define COMMAND_IR_FINDER3_SENDING_ANGLE_AND_DISTANCE 0x09

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes
uint16_t reading[12];
bool near;

void setup() {
  Serial.begin(115200);
  delay(1000);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x330 + 2 * i, 1, data);
}

void loop() {
  uint8_t startIndex = 0;
  // Receive a message
  CANBusMessage* msg = can.messageReceive();
  if (msg != NULL){
    int16_t angle = ((msg->data[1] << 8) | msg->data[2]) - 180;
    uint16_t distance = (msg->data[3] << 8) | msg->data[4];
    near = msg->data[5];

    // Display results
    static uint32_t ms = 0;
    if (millis() - ms > 500){
      Serial.print(angle);
      Serial.print(" deg, strength: ");
      Serial.print(distance);
      Serial.println(near ? " near" : " far");
      ms = millis();
    }
  }
}

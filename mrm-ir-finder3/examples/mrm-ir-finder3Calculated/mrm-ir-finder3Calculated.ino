#include <mrm-can-bus.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA 0x15  // Command to instruct the sensor to send calculated data.
#define COMMAND_IR_FINDER3_SENDING_ANGLE_AND_DISTANCE 0x09                  // Command indicating that the payload consists of angle and distance instead of each receiver's value.

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes.
uint16_t reading[12];   // Array to hold all the readings (12 receivers).
bool near;              // Ball is near.

void setup() {
  Serial.begin(115200);                           // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
  delay(1000);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA;  // First byte of the content. We will order the sensor to start sending calculated data.
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x330 + 2 * i, 1, data);      // 0x330 is sensor 1, 0x332 sensor 2, etc. This loop will start all the sensors that are connected to the bus.
}

void loop() {
  uint8_t startIndex = 0;
  // Receive a message
  CANBusMessage* msg = can.messageReceive();                    // Receive a message, if any arrived.
  if (msg != NULL){                                             // If not NULL, a message received.
    int16_t angle = ((msg->data[1] << 8) | msg->data[2]) - 180; // Reconstruct angle from 2 bytes.
    uint16_t distance = (msg->data[3] << 8) | msg->data[4];     // Reconstruct distance from 2 bytes.
    near = msg->data[5];                                        // One of the remaining bytes is use to carry information if the ball is near.

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

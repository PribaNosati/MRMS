#include <mrm-can-bus.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10   // Command that instruct the sensor to start sending readings.
#define COMMAND_SENSORS_MEASURE_SENDING 0x13      // Command indicating that the payload is sensor's reading.

Mrm_can_bus can;

uint8_t data[8];                                  // Message content: 8 bytes

void setup() {
  Serial.begin(115200);                           // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
  delay(1000);
  Serial.println("Start");

  // Start all the sensors connected to the CAN Bus.
  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x370 + 2 * i, 1, data);     // 0x370 is sensor 1 of the first range, 0x372 sensor 2, etc. This loop will start all the sensors that are connected to the bus.
}

void loop() {
  CANBusMessage* msg = can.messageReceive();                            // Receive a message
  if (msg != NULL && msg->data[0] == COMMAND_SENSORS_MEASURE_SENDING){  // If not NULL, a message received. Also check the command to see if the payload is a reading.
    uint16_t mm = (msg->data[2] << 8) | msg->data[1];                   // Reconstruct milimmeters.
    Serial.print(mm);
    Serial.println(" mm");
  }
}

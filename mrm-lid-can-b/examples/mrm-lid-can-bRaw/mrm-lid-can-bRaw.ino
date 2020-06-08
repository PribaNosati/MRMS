#include <mrm-can-bus.h>

#define COMMAND_LID_CAN_B_CALIBRATE 0x05
#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_ONCE 0x11
#define COMMAND_SENSORS_MEASURE_STOP 0x12
#define COMMAND_SENSORS_MEASURE_SENDING 0x13
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_REQUEST_NOTIFICATION 0x14
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_AND_RETURN_CALCULATED_DATA 0x15
#define COMMAND_SENSORS_MEASURE_CALCULATED_SENDING 0x16
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_2 0x17
#define COMMAND_SENSORS_MEASURE_CONTINUOUS_VERSION_3 0x18
#define COMMAND_FIRMWARE_REQUEST 0x19
#define COMMAND_FIRMWARE_SENDING 0x1A
#define COMMAND_RESET 0x1B
#define COMMAND_MESSAGE_SENDING_1 0x1C
#define COMMAND_MESSAGE_SENDING_2 0x1D
#define COMMAND_MESSAGE_SENDING_3 0x1E
#define COMMAND_MESSAGE_SENDING_4 0x1F
#define COMMAND_FPS_REQUEST 0x30
#define COMMAND_FPS_SENDING 0x31
#define COMMAND_ID_CHANGE_REQUEST 0x40
#define COMMAND_NOTIFICATION 0x41
#define COMMAND_ERROR 0xEE
#define COMMAND_REPORT_ALIVE 0xFF

Mrm_can_bus can;

uint8_t data[8];        // Message content: 8 bytes

void setup() {
  Serial.begin(115200);
  delay(1000);

  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;  // First byte of the content
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x180 + 2 * i, 1, data);
  for (uint8_t i = 0; i < 8; i++)
    can.messageSend(0x280 + 2 * i, 1, data);
}

void loop() {
  // Receive a message
  CANBusMessage* msg = can.messageReceive();
  if (msg != NULL && msg->data[0] == COMMAND_SENSORS_MEASURE_SENDING){
    uint16_t mm = (msg->data[2] << 8) | msg->data[1];
    Serial.print(mm);
    Serial.println(" mm");
  }
}
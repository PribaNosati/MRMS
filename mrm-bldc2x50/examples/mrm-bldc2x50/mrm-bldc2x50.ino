#include <mrm-can-bus.h>

#define ID_MOTOR1 0x110
#define ID_MOTOR2 0x112

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_SENDING 0x13
#define COMMAND_SPEED_SET 0x20

uint32_t motorId[2] = {ID_MOTOR1, ID_MOTOR2};
Mrm_can_bus can;
uint8_t data[8];        // Message content: 8 bytes

void setSpeed(uint8_t motorNumber, int8_t speed) {
  data[0] = COMMAND_SPEED_SET;
  data[1] = speed + 128;
  can.messageSend(motorId[motorNumber], 2, data);
}


void setup() {
  Serial.begin(115200);
  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
  for (uint16_t motor = 0; motor < 2; motor++)
    can.messageSend(motorId[motor], 1, data);
}

void loop() {
  for (uint16_t motor = 0; motor < 2; motor++){    
    int8_t step = 1;
    for (int8_t speed = 1; !(speed == 0 && step == 1); speed += step){
      setSpeed(motor, speed);
      uint32_t ms = millis();
      while (millis() - ms < 30){
        // Receive a message
        CANBusMessage* msg = can.messageReceive();
        if (msg != NULL && msg->messageId == motorId[motor] + 1 && msg->data[0] ==  COMMAND_SENSORS_MEASURE_SENDING){
          uint32_t enc = (msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1];
          Serial.println(enc);
        }
      }
      if (abs(speed) == 127)
        step = -step;
    }
  }
}
#include <mrm-can-bus.h>

#define ID_MOTOR1 0x260
#define ID_MOTOR2 0x262

#define COMMAND_SPEED_SET 0x20

uint32_t motorId[2] = {ID_MOTOR1, ID_MOTOR2};
Mrm_can_bus can();
uint8_t data[8];        // Message content: 8 bytes

void setSpeed(uint8_t motorNumber, int8_t speed) {
  data[0] = COMMAND_SPEED_SET;
  data[1] = speed + 128;
  can.messageSend(motorId[motorNumber], 2, data);
}


void setup() {
}

void loop() {
  for (uint16_t motor = 0; motor < 2; motor++){    
    int8_t step = 1;
    for (int8_t speed = 1; !(speed == 0 && step == 1); speed += step){
      setSpeed(motor, speed);
      delay(30);
      if (abs(speed) == 127)
        step = -step;
    }
  }
}

#include <mrm-can-bus.h>

#define ID_MOTOR1 0x260                         // Address of the 1. motor
#define ID_MOTOR2 0x262                         // Address of the 2. motors

#define COMMAND_SPEED_SET 0x20                  // CAN Bus command to set motor's speed.

uint32_t motorId[2] = {ID_MOTOR1, ID_MOTOR2};   // Array to handle motors in a loop later.
Mrm_can_bus can;                              // CAN Bus object.
uint8_t data[8];                                // Message content: 8 bytes

// Function that sets motor's speed for 1 of the possible 2 motors.
void setSpeed(uint8_t motorNumber, int8_t speed) {
  data[0] = COMMAND_SPEED_SET;
  data[1] = speed + 128;                        // Change to positive integer to satisfy the command's format.
  can.messageSend(motorId[motorNumber], 2, data); // Send the message. That will cause the motor to start spinning at the selected speed.
}


void setup() {
    Serial.begin(115200);                           // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
}

void loop() {
  for (uint16_t motor = 0; motor < 2; motor++){     // For both motors.    
    int8_t step = 1;
    for (int8_t speed = 1; !(speed == 0 && step == 1); speed += step){// This loop changes speed gradually. The logic is not important for the purpose of this exercise.
      Serial.println(speed);
      setSpeed(motor, speed);                       // Set the motor's speed.
      delay(30);
      if (abs(speed) == 127)
        step = -step;
    }
  }
}

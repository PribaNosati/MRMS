#include <mrm-can-bus.h>

#define ID_MOTOR1 0x250
#define ID_MOTOR2 0x252
#define ID_MOTOR3 0x254
#define ID_MOTOR4 0x256

#include <mrm-can-bus.h>

#define ID_MOTOR1 0x250                           // Address of the 1. motor
#define ID_MOTOR2 0x252                           // Address of the 2. motor
#define ID_MOTOR3 0x254                           // Address of the 3. motor
#define ID_MOTOR4 0x256                           // Address of the 4. motor

#define COMMAND_SPEED_SET 0x20                    // CAN Bus command to set motor's speed.

uint32_t motorId[4] = {ID_MOTOR1, ID_MOTOR2, ID_MOTOR3, ID_MOTOR4}; // Array to iterate motors in loops.
Mrm_can_bus can;                                  // CAN Bus object.
uint8_t data[8];                                  // Message content: 8 bytes

// Function that sets motor's speed for 1 of the possible 2 motors.
void setSpeed(uint8_t motorNumber, int8_t speed) {
  data[0] = COMMAND_SPEED_SET;                    // First byte of the CAN Bus message: command.
  data[1] = speed + 128;                          // Function that sets motor's speed for 1 of the possible 2 motors.
  can.messageSend(motorId[motorNumber], 2, data); // Send the message. That will cause the motor to start spinning at the selected speed.
}


void setup() {
    Serial.begin(115200);                           // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
}

void loop() {
  for (uint16_t motor = 0; motor < 4; motor++){   // For all the 4 motors motors...
    int8_t step = 1;
    for (int8_t speed = 1; !(speed == 0 && step == 1); speed += step){ // This loop changes speed gradually. The logic is not important for the purpose of this exercise.
      setSpeed(motor, speed);                     // Set the motor's speed.
      delay(30);
      if (abs(speed) == 127)
        step = -step;
    }
  }
}

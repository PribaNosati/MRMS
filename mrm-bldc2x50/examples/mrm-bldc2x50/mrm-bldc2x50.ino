#include <mrm-can-bus.h>

#define ID_MOTOR1 0x110                         // Address of the 1. motor
#define ID_MOTOR2 0x112                         // Address of the 2. motors

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10 // CAN Bus command that starts encoders monitoring
#define COMMAND_SENSORS_MEASURE_SENDING 0x13    // CAN Bus command header that indicates that the payload consists of measurements.
#define COMMAND_SPEED_SET 0x20                  // CAN Bus command to set motor's speed.

uint32_t motorId[2] = {ID_MOTOR1, ID_MOTOR2};   // Array to handle motors in a loop later.
Mrm_can_bus can;                                // CAN Bus object.
uint8_t data[8];                                // Message content: 8 bytes

// Function that sets motor's speed for 1 of the possible 2 motors.
void setSpeed(uint8_t motorNumber, int8_t speed) {
  data[0] = COMMAND_SPEED_SET;
  data[1] = speed + 128;                        // Change to positive integer to satisfy the command's format.
  can.messageSend(motorId[motorNumber], 2, data); // Send the message. That will cause the motor to start spinning at the selected speed.
}


void setup() {
  Serial.begin(115200);                         // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
  data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS; // Command to order the controller to start sending encoder counts.
  for (uint16_t motor = 0; motor < 2; motor++)  // For both motors.
    can.messageSend(motorId[motor], 1, data);   // Let the motor start sending encoder's data.
}

void loop() {
  for (uint16_t motor = 0; motor < 2; motor++){     // For both motors.
    int8_t step = 1;
    for (int8_t speed = 1; !(speed == 0 && step == 1); speed += step){ // This loop changes speed gradually. The logic is not important for the purpose of this exercise.
      setSpeed(motor, speed);                       // Set the motor's speed.
      uint32_t ms = millis();
      while (millis() - ms < 30){                   // During the next 30 ms...
        CANBusMessage* msg = can.messageReceive();  // Receive a message, if available
        // If a message received (msg != NULL), and it is for the current motor, and its payload sends encoders...
        if (msg != NULL && msg->messageId == motorId[motor] + 1 && msg->data[0] ==  COMMAND_SENSORS_MEASURE_SENDING){
          // Reassemble the encoder's payload into an integer (enc).
          uint32_t enc = (msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1];
          Serial.println(enc);                      // Print the encoder count.
        }
      }
      if (abs(speed) == 127)
        step = -step;
    }
  }
}

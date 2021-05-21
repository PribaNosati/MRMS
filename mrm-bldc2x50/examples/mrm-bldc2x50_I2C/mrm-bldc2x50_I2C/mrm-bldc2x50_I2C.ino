#include <Wire.h>                                     // I2C library.

#define DATA_COUNT 1                                  // Number of bytes to receive.
#define I2C_ADDRESS 0x12                              // Driver's I2C address.
#define REGISTER_FIRMWARE 0x01                        // I2C virtual register containing firmware version.
#define REGISTER_ALL_SPEEDS 0x00                      // I2C virtual register for settings speeds

uint32_t ms;                                          // Variable used for displaying results periodically.

void setup() {
  Serial.begin(115200);                               // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
  Wire.begin();                                       // Start I2C.
  delay(500);                                         // Wait to be sure that the text will be displayed.
  Serial.println("Start");

  Wire.beginTransmission(I2C_ADDRESS);                // Start a "write" I2C command (write to driver).
  Wire.write((uint8_t)REGISTER_FIRMWARE);             // Send the desired virtual register address.
  Wire.endTransmission();                             // End "write" part.

  Wire.requestFrom((int)I2C_ADDRESS, DATA_COUNT);     // Start "read" part (read from sensor). Request 4 bytes of data.
  uint8_t fw = Wire.read();                           // Read firmware version.
   
  Serial.println("FW: " + (String)fw + ".");          // Display.
}

void loop() {
  int8_t step = 1;
    for (int8_t speed = 1; !(speed == 0 && step == 1); speed += step){ // This loop changes speed gradually. The logic is not important for the purpose of this exercise.
      Wire.beginTransmission(I2C_ADDRESS);            // Start a "write" I2C command (write to sensor).
      Wire.write((uint8_t)REGISTER_ALL_SPEEDS);       // Send the desired virtual register address.
      for (uint8_t i = 0; i < 2; i++)                 // For all the 4 motors...
        Wire.write((uint8_t)speed + 128);             // Set speed. It is the same for all the motors. Add 128 to the speed to get the format needed for this command.
      Wire.endTransmission();                         // End "write" part.
      delay(30);                                      // Wait 30 ms in order not to change the speeds abruptly.
    
      if (millis() - ms > 100){                       // Every 0.1 sec...   
        Serial.println("Speed: " + (String)speed + "."); // Display.
        ms = millis();
      }
      
      if (abs(speed) == 127)                          // Change rotation direction.
        step = -step;
   }
}

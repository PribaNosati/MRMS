#include <Wire.h>                                     // I2C library.

#define DATA_COUNT 4                                  // Number of bytes to receive.
#define I2C_ADDRESS 0x10                              // Sensor's I2C address.
#define REGISTER_ANGLE_AND_DISTANCE 0x00              // I2C virtual register with distance and angle.

uint32_t ms;                                          // Variable used for displaying results periodically.

void setup() {
  Serial.begin(115200);                               // Adjust monitor speed to 115200 bps in order not to get garbage in the window.
  Wire.begin();                                       // Start I2C.
  delay(500);                                         // Wait to be sure that the text will be displayed.
  Serial.println("Start");
}

void loop() {
  if (millis() - ms > 100){                           // Every 0.1 sec...

    Wire.beginTransmission(I2C_ADDRESS);              // Start a "write" I2C command (write to sensor).
    Wire.write((uint8_t)REGISTER_ANGLE_AND_DISTANCE); // Send the desired virtual register address.
    Wire.endTransmission();                           // End "write" part.

    Wire.requestFrom((int)I2C_ADDRESS, DATA_COUNT);   // Start "read" part (read from sensor). Request 4 bytes of data.
    uint16_t angle = Wire.read() << 8;                // Here and in the next 3 lines, read 4 bytes and reconstruct 2 16-bit values: angle and distance.
    angle |= Wire.read();
    uint16_t distance = Wire.read() << 8;
    distance |= Wire.read();
    
    Serial.println("Distance: " + (String)distance + ", angle: " + (String)angle + "°"); // Display.

    ms = millis();
  }
}

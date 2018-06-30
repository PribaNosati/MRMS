#include "UltrasonicSRF08s.h"
#include <i2c_t3.h>

UltrasonicSRF08s us;

void setup()
{
  Serial.begin(115200);// Start communication with a computer connected to Arduino via a USB cable
  Wire.begin(); // Start I2C

  //Add one Devantech SRF08 sensor
  us.add(0x70);

  //Test
  us.test();
}

void loop(){}

/** If something goes wrong, it will be the best to stop the program.
*/
void error(String message) {
  Serial.print(message);
  while (true)
    ;
}

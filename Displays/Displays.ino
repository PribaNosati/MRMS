#include <i2c_t3.h>
#include "Displays.h"

Displays displays;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(500);

  displays.add(0x70);
  displays.test(0);
}

void loop(){}

void error(String message){
  Serial.println(message);
  while(1);
}

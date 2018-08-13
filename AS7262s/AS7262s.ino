#include "AS7262s.h"

AS7262s colorSensors; //Object which contains all the color sensors

void setup()
{
  Serial.begin(115200); // Serial interface to Your PC
  Wire.begin(); // I2C starts.
  delay(1000); // Wait for Serial.
  Serial.println("Start");

  colorSensors.add(); // Add a color sensor using all defaults.
  colorSensors.test(); 
}

void loop(){}

void error(String message){
  Serial.println(message);
  while(1);
}

#include <mrm-imu.h>

Mrm_imu imu;

void setup(){
  Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable
  Wire.begin();         // Start I2C
  imu.add(true); 
}

void loop(){
  Serial.print("Heading: ");
  Serial.print(imu.heading());
  Serial.print(", pitch: ");
  Serial.print(imu.pitch());
  Serial.print(", roll: ");
  Serial.println(imu.roll());
  delay(100);
}

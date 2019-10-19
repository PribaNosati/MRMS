#include <Wire.h>

uint8_t address = 0x7C;

void setup()
{
  Wire.begin(); // Start I2C bus
  Serial.begin(115200); // Start communication with a computer connected to Arduino via a USB cable
  delay(500); // Wait for serial communication to start
  Serial.println("I2C scanner");
}


void loop()
{
	Serial.println(millis());
    /*Wire.beginTransmission(address); // Transmission tried
    byte status = Wire.endTransmission(); // Was it successful?
    if (status == 0)
      Serial.print(" Found");
	  delay(1);*/
	  
	  Wire.beginTransmission(address);
	  Wire.write(0x93);
	  uint8_t status = Wire.endTransmission();
	  if (status == 0)
		  Serial.println(", TX OK");
	  else
		  Serial.println(", TX failed");

    /*  Wire.requestFrom((uint8_t)address, (uint8_t)1);                // Request 2 bytes from SRF module
      uint32_t startMs = millis();
      bool any = false;
      uint8_t received = 0; 
      while(millis() - startMs < 1000){
        if (Wire.available() >=1){                     // Wait for data to arrive
          any = true;
          received = Wire.read();
        }
      }
      if (any)
        Serial.println("Arrived: " + (String)received);
        else
        Serial.println("Timeout");
      Wire.endTransmission();*/
      
		/*Wire.beginTransmission(SRF_ADDRESS);             // Start communticating with SRF08
		  Wire.write(CMD);                                 // Send Command Byte
		  Wire.write(0x51);                                // Send 0x51 to start a ranging
		  Wire.endTransmission();
		  
		  Wire.requestFrom(SRF_ADDRESS, 2);                // Request 2 bytes from SRF module
		  while(Wire.available() < 2);                     // Wait for data to arrive
		  highByte = Wire.read();                          // Get high byte
		  lowByte = Wire.read();                           // Get low byte*/
	  
	/*}
    else 
      Serial.println(" Nothing");*/

  delay(1000); 
}

/**
Purpose: UART communication example
@author MRMS team
@version 0.0 2018-08-08
Licence: You can use this code any way you like.
*/

#include "UART.h" //Change into <UART.h> in Your code

UART uart;
Message message;

void setup(){
  Serial.begin(115200);

  message.append((uint8_t)0);
  uart.write(message);
}

void loop(){
  doSomethingUseful();
  handleMessages();
}

void doSomethingUseful(){
  delay(200);
}

void error(String message){
  Serial.println(message);
  while(true)
    ;
}

void handleMessages(){
   if (uart.available()){
    message = uart.readMessage();
    uint8_t messageId = message.readUInt8();
    switch(message[0]){
      case 1:{
          uint16_t x = message.readUInt16();
          Serial.println("x = " + (String)x);
        }
        break;
      default:
        error("Impossible command id: " + (String)(int)messageId);
        break;
    }
  }
}

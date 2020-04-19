// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ESP32SJA1000.h>

#define COMMAND_SENSORS_MEASURE_CONTINUOUS 0x10
#define COMMAND_SENSORS_MEASURE_STOP 0x12
#define COMMAND_SENSORS_MEASURE_SENDING 0x13

uint8_t rx_length;
uint32_t rx_stdId;
uint8_t rx_data[8];
uint8_t tx_data[8];

void messageSend(uint32_t stdId, uint8_t dlc, uint8_t data[8]) {
    if (!CAN.beginPacket(stdId, dlc)){
      Serial.println("Error sending");
      while(1);
    }
    if (CAN.write(data, dlc) != dlc){
      Serial.println("Wrong byte count");
      while(1);
    }
    CAN.endPacket();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("CAN Receiver Callback");

  // start the CAN bus at 250 kbps
  if (!CAN.begin(250E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
    
  // register the receive callback
  CAN.onReceive(onReceive);

  tx_data[0] = COMMAND_SENSORS_MEASURE_CONTINUOUS;
  for (uint8_t i = 0; i < 8; i++)
    messageSend(0x150 + 2*i, 1, tx_data);
}

void loop() {
  // do nothing
}

void onReceive(int packetSize) {
  // received a packet
  rx_stdId = CAN.packetId();
  rx_length = packetSize;
  
  Serial.print("Received packet with id 0x");
  Serial.print(rx_stdId, HEX);


    Serial.print(" and length ");
    Serial.print(rx_length);
    Serial.print(": ");

    uint8_t i = 0;
    while (CAN.available()) {
      rx_data[i++] = CAN.read();
      Serial.print(rx_data[i-1], HEX);
      Serial.print(" ");
    }
    Serial.println();

  Serial.println();
}

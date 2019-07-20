#include <mrm-ref-can_esp32.h>

Mrm_ref_can mrm_ref_can;

#define CAN_ID_REF_CAN_IN 0x0160
#define CAN_ID_REF_CAN_OUT 0x0161

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Start");

  mrm_ref_can.add(CAN_ID_REF_CAN_IN, CAN_ID_REF_CAN_OUT);
  mrm_ref_can.test();  
}

void loop() {
}

void error(String message){
  printf("%s", message);
}

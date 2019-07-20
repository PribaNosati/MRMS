#include <mrm-lid-can-b.h>

Mrm_lid_can_b mrm_lid_can_b;

#define CAN_ID_VL531X_IN 0x0150
#define CAN_ID_VL531X_OUT 0x0151

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Start");

  mrm_lid_can_b.add(CAN_ID_VL531X_IN, CAN_ID_VL531X_OUT);
  mrm_lid_can_b.test();  
}

void loop() {
}

void error(String message){
  printf("%s", message);
}

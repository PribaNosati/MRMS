#include <i2c_t3.h>
#include "Displays.h"

Displays displays;

static const uint8_t PROGMEM
BITMAP1[] =
{
  B00011000,
  B00111100,
  B01111110,
  B11111111,
  B00000000,
  B00000000,
  B00000000,
  B00000000
},
BITMAP2[] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00111100,
  B00111100,
  B00111100,
  B00111100
};

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  delay(500);

  displays.add(0x70);

  displays.displayChar(0, 'S');
  delay(1000);
  displays.displayChar(0, 'S', LED_RED);
  delay(1000);

  displays.clear(0);
  delay(500);

  displays.drawBitmap(0, 0, 0, BITMAP2, 8, 8, LED_YELLOW, true);
  delay(1000);

  displays.drawBitmap(0, 0, 0, BITMAP1);
  displays.drawBitmap(0, 0, 0, BITMAP2, 8, 8, LED_RED);
  displays.writeDisplay(0);
  delay(1000);
  
  displays.test(0);
}

void loop(){}

void error(String message){
  Serial.println(message);
  while(1);
}

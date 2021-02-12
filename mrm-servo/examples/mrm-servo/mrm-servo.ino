#include <mrm-servo.h>

// Angle is being increased from 0 counterclockwise to 300 degrees
#define MAXIMUM_DEGREES 300                       // Depends on servo model.
#define SET_DEGREES 290                           // Move servo to this predefined angle.
#define SWEEP 0                                   // 0 - sweep, 1 position servo at SET_DEGREES.

Mrm_servo mrm_servo;                              // Object representing servo.
uint16_t degrees = 0;                             // Current angle.

void setup() {
  mrm_servo.add(18, "Servo1", 0, 300, 0.5, 2.5);  // Connect servo to GPIO 18.
#if !SWEEP
  mrm_servo.write(SET_DEGREES);                   // Position servo.
  while(1);                                       // Stop the program and do not enter loop().
#endif
}

void loop() {
  mrm_servo.write(degrees);                       // Postion servo.
  delay(10);
  if (++degrees == MAXIMUM_DEGREES)               // First increase by and and then compare to maximum.
    degrees = 0;                                  // Reset and sweep again.
}

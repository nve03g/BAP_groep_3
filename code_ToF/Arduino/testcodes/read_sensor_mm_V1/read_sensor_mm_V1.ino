/* TESTCODE:
testing sensor measurement accuracy */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor; // creates a VL53L0X object

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {} // this infinite loop ends the program before entering main loop
  }

  sensor.startContinuous(); // take readings as fast as possible, or provide desired inter-measurement period in ms
}

void loop()
{
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}

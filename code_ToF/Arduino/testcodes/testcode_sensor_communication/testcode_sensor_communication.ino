/* TESTCODE:
testing whether the sensor values are properly transferred to Python */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor; // creates a VL53L0X object

int position_data;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor! Trying again...");
    while (!sensor.init()) {
      sensor.init(); // keep trying to initiate sensor
    }
    Serial.println("Succes! Sensor now initialized.");
  }

  sensor.startContinuous(); // take readings as fast as possible, or provide desired inter-measurement period in ms
}

void loop() {
  position_data = sensor.readRangeContinuousMillimeters();
	Serial.write((0)&position_data, sizeof(position_data)); // transfer sensor data in bytes over serial communication line
}
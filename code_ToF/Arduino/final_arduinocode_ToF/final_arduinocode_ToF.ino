#include <Wire.h>
#include <VL53L0X.h>

int readData;
int pwmPin = 3;

VL53L0X sensor; // creates a VL53L0X object

unsigned long time_now = 0;
unsigned long time_fans = 0;
unsigned long time_TOF = 0;
int data_old = 0;

// typedef union {
//   float floatingPoint;
//   byte binary[4];
// } binaryFloat;

//binaryFloat data_pos;
int data_pos;

void setup(){
  Serial.begin(115200); // change to 115200?
  Wire.begin();

  pinMode(pwmPin, OUTPUT);

  sensor.setTimeout(500);
  if (!sensor.init()){
    // Serial.println("Failed to detect and initialize sensor! Trying again...");
    while (!sensor.init()) {
      sensor.init(); // try initializing sensor
    }
    // Serial.println("Succes! Sensor now initialized.");
  }

  sensor.startContinuous(); // take readings as fast as possible, or provide desired inter-measurement period in ms

  time_now = millis();
  time_fans = time_now;
}

void loop(){
// get PWM value to set fan
  if (Serial.available() > 0){
    readData = Serial.read(); // read incoming char (PWM value)
    //convData = map(readData,0,100,0,255); // convert % to effective PWM value MAP ENKEL MET INT; now not needed, bcs python code sends PWM value directly, not in %
  }
// set fan to needed PWM value
  time_now = millis();
  if (time_now - time_fans > 10){ // 2 ms
    if (data_old != readData){
      analogWrite(pwmPin, readData);
      data_old = readData;
      time_fans = millis();
    }
  }

	data_pos = sensor.readRangeContinuousMillimeters();
	Serial.write((byte*)&data_pos, sizeof(data_pos));
}
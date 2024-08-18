int readData;
int pwmPin = 3;

unsigned long time_now = 0;
unsigned long time_fans = 0;

int data_old = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Ready");
  
  pinMode(pwmPin, OUTPUT);
  
  time_now = millis();
  time_fans = time_now;
}

void loop() {
  if (Serial.available() > 0){
    readData = Serial.read(); // read incoming char (PWM value)
  }
  time_now = millis();
  if (time_now - time_fans > 10){ // 10 ms
    if (data_old != readData){
      analogWrite(pwmPin, readData);
      data_old = readData;
      time_fans = millis();
    }
  }
  
}

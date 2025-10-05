// Receive block data from Raspberry Pi, send block data to Raspberry Pi
// Quelle: https://forums.raspberrypi.com/viewtopic.php?t=203286

#include <Wire.h>

#define SLAVE_ADDRESS 0x08

byte b[4] = {210, 220, 230, 240};
// data buffer
int data[9];

void receiveData(int byteCount){
  int counter = 0;
  while(Wire.available()) {
      data[counter] = Wire.read();
      counter ++;
  }
  Serial.print("Got data: ");
  for(counter=0; counter<byteCount; counter++){
    Serial.print(data[counter]);
    Serial.print(" ");
  }
  Serial.println();
}

void sendData(){
    b[3] ++;
    b[0] ++;
    Wire.write(b, 4);
    Serial.println("data sent");
}

void setup(){
  Serial.begin(115200); // start serial for output
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("I2C Ready!");
}

void loop(){
  //delay(1000);
}

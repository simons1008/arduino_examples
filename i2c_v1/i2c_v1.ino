/* 
 * Arduino code to send and receive I2C data
 * Tested on Adafruit Feather M0+ Express and Raspberry Pi Model 4
 * 
 * SDA <--> SDA
 * SCL <--> SCL
 * GND <--> GND
 * 
 * Sets built-in LED (1 = on, 0 = off) on Feather when requested 
 * and responds with data received
 */
// Quelle: https://community.element14.com/products/arduino/b/blog/posts/raspberry-pi-and-arduino-i2c-communication
// Geändert: Serial output ergänzt
//           LED leuchtet wenn i2cData == 255
// Getestet mit Arduino UNO R3 und Raspberry Pi 4 B 

#include <Wire.h>
#define SLAVE_ADDRESS 0x04       // I2C address for Arduino
#define LED 13                   // Built-in LED
int i2cData = 0;                 // the I2C data received
void setup(){
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(115200);         // Serial output
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}
void loop() {
  // Everything happens in the interrupts
}
// Handle reception of incoming I2C data
void receiveData(int byteCount) {
  while (Wire.available()) {
    i2cData = Wire.read();
    Serial.print("i2cData:");
    Serial.println(i2cData);
    if (i2cData == 255) {
      digitalWrite(LED, 1);
    }
    else {
      digitalWrite(LED, 0);
    }
  }
}
// Handle request to send I2C data
void sendData() { 
  Wire.write(i2cData);
}
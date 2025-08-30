/************************************************************************
 * Show encoder signals ENCA and ENCB when motor shaft is turned by hand
 *
 ************************************************************************/

/* Quelle: https://github.com/curiores/ArduinoTutorials/tree/main/encoderControl/part1 */

#define ENCA 2
#define ENCB 3

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
}

void loop() {
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  Serial.print("ENCA:");
  Serial.print(a*5); 
  Serial.print(":");
  Serial.print("ENCB:");
  Serial.print(b*5);
  Serial.println();
}

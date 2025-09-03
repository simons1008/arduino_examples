/*************************************************************************
 * The PID Controller controls the speed of the "DC Motor with Encoder"
 * The velocity is calculated with method1 and method2.
 * method1 calculates the velocity in the loop() function.
 * method2 calculates the velocity in the interrupt service routine.
 * method2 is more accurate than method1. Only method1 can measure zero velocity.
 * PID Control data can be recorded with both methods:
 * - with method1 calculated velocity 
 * - with method2 calculated velocity and subsequent low pass filtering
 * The target velocity vt is a rectangular function. 
 *************************************************************************/

/************************************************************************
 * Quelle: https://github.com/curiores/ArduinoTutorials/tree/main/SpeedControl
 * Geändert: Zeile      31: Gilt für Motor "Waveshare N20 DC Getriebemotor"
 *           Zeile  81/ 82: RPM Berechnung mit den Eigenschaften des Motors
 *           Zeile  91/ 92: Die Rechteckfunktion schwingt um Null
 *           Zeile      98: Optionale Berechnung mit Methode2
 *           Zeile 115/118: Variablennamen mit Doppelpunkt vorangestellt
 ************************************************************************/
#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

// motor characteristics
float PPR = 300.0; // 16 encoder poles x 18.75 gear ratio

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/PPR*60.0;
  float v2 = velocity2/PPR*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float a = 100; 
  float vt = 2*a*(sin((6.28/20)*currT/1e6)>0) - a; // period 20 seconds

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
  // float e = vt - v2; 
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  
  setMotor(dir,pwr,PWM,IN1,IN2);

  Serial.print("vt:");
  Serial.print(vt);
  Serial.print(":");
  Serial.print("v1Filt:");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}

/**************************************************************************
 * The PID Controller controls the position of the "DC Motor with Encoder".  
 * The target position is a step function or a sinusoidal function  
 **************************************************************************/

/************************************************************************** 
 * Quelle: https://github.com/curiores/ArduinoTutorials/tree/main/encoderControl/part4 
 * Geändert: Zeile   19/20: Kabelfarben entfernt
 *           Zeile      31: Baudrate erhöht
 *           Zeile      48: neue Zielfunktion: periodische Sprungfunktion
 *           Zeile   51/52: kp und kd erhöht
 *           Zeile      71: Korrektur: Zielgröße positiv, Rückführung negativ 
 *             (die Autorin hatte die Motorleitungen verdreht angeschlossen)
 *           Zeile 102-105: Serial Output mit Doppelpunkt (für EXCEL)
 *           Zeile     106: delay() steuert die Abtastrate  
 **************************************************************************/

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  //int target = 250*sin(prevT/1e6);
  int target = 3000*(sin(6.28/20*prevT/1e6)>0); // period 20 seconds

  // PID constants
  float kp = 1.2;
  float kd = 0.2;
  float ki = 0.0;


  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  //int e = pos - target;
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print("target:");
  Serial.print(target);
  Serial.print(":");
  Serial.print("pos:");
  Serial.print(pos);
  Serial.println();
  delay(20); // controls the sampling rate
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

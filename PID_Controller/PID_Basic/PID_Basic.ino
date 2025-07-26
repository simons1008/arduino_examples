/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
 /* Quelle: Arduino Library PID by Brett Beauregard - Examples */

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
// double Kp=2, Ki=5, Kd=1;
double Kp=0, Ki=1, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);

// Setpoint Array
double mySetpoint[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100,  50, 50, 50, 50, 50,  0};

// Input Array
double myInput[] =    {60, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 90, 110, 90, 110, 90, 40, 60, 40, 60, 40};
// double myInput[] =    {50, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 100, 100, 100, 100, 100, 50, 50, 50, 50, 50};

// Array Index
int index = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("PID Test");
  //initialize the variables we're linked to
  // Input = analogRead(PIN_INPUT);
  Input = myInput[index]; 

  // Setpoint = 100;
  Setpoint = mySetpoint[index]; 
  
  // Sample Time in Milliseconds
  myPID.SetSampleTime(1000);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  // Input = analogRead(PIN_INPUT);
  if (myPID.Compute())
  {
    // Setpoint
    Serial.print("Setpoint:");
    Serial.print(Setpoint); 
    Serial.print(",");
    // Input
    Serial.print("Input:");
    Serial.print(Input);
    Serial.print(",");
    // Output
    Serial.print("Output:");
    Serial.println(Output);
    // neuer Input
    index ++;
    if (index > 19)
      index = 0;
    Input = myInput[index]; 
    Setpoint = mySetpoint[index];
  }
  // analogWrite(PIN_OUTPUT, Output);
}



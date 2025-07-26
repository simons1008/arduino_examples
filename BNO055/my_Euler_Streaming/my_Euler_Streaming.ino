/*
 ***************************************************************************

    Euler_Streaming.pde - part of sample SW for using BNO055 with Arduino

   (C) All rights reserved by ROBERT BOSCH GMBH

   Copyright (C) 2014 Bosch Sensortec GmbH

 	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/
/*	Date: 2014/01/07
 	 Revision: 1.2

*/

#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>
#include <SoftwareSerial.h> 

// conditional compilation
//#define DEBUG

//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_quaternion myQuaternionData; // Structure to hold Quaternion Data

unsigned long lastTime = 0;

// variables hold calibration status
unsigned char accelCalibStatus = 0;	
unsigned char magCalibStatus = 0;	
unsigned char gyroCalibStatus = 0;	
unsigned char sysCalibStatus = 0;	
unsigned char calibStatus = 0; 

// serial data output interface
SoftwareSerial mySerial(10, 11);

void setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to operation mode
   bno055_set_operation_mode(OPERATION_MODE_NDOF);
  //bno055_set_operation_mode(OPERATION_MODE_COMPASS);

  delay(1);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(9600);

  //Initialize the Serial Port for data output
  mySerial.begin(9600);

  // initialize calib_status
  calibStatus = 0; 
  
  // wait until every sensor calibration returns 3
  while(calibStatus != 3)
  {
    // Get sensor calibration status
		bno055_get_accelcalib_status(&accelCalibStatus); //To read out the Accelerometer Calibration Status (0-3)
		bno055_get_magcalib_status(&magCalibStatus); //To read out the Magnetometer Calibration Status (0-3)
		bno055_get_gyrocalib_status(&gyroCalibStatus); //To read out the Gyroscope Calibration Status (0-3)
		bno055_get_syscalib_status(&sysCalibStatus); //To read out the System Calibration Status (0-3)
    
    // Message to serial monitor
    Serial.println("waiting for sensor calibration");

#ifndef DEBUG    
    // print sensor calibration status
		mySerial.print("accelCalibStatus:");
		mySerial.print(accelCalibStatus);
    mySerial.print(",");
		mySerial.print("magCalibStatus:");
		mySerial.print(magCalibStatus);
    mySerial.print(",");
		mySerial.print("gyroCalibStatus:");
		mySerial.print(gyroCalibStatus);
    mySerial.print(",");
		mySerial.print("sysCalibStatus:");
		mySerial.println(sysCalibStatus);
#else 
    // print sensor calibration status
		Serial.print("accelCalibStatus:");
		Serial.print(accelCalibStatus);
    Serial.print(",");
		Serial.print("magCalibStatus:");
		Serial.print(magCalibStatus);
    Serial.print(",");
		Serial.print("gyroCalibStatus:");
		Serial.print(gyroCalibStatus);
    Serial.print(",");
		Serial.print("sysCalibStatus:");
		Serial.println(sysCalibStatus);
#endif
    // 1 second pause
    delay(1000);

    // bitwise AND of sensor calibration status
    calibStatus = accelCalibStatus & magCalibStatus & gyroCalibStatus & sysCalibStatus;
  }
}

void loop() //This code is looped forever
{
  if ((millis() - lastTime) >= 200) //To stream at 5Hz without using additional timers
  {
    lastTime = millis();

    //Update Euler data into the structure
    bno055_read_euler_hrp(&myEulerData);

#ifndef DEBUG
    mySerial.print("TimeStamp:");				//To read out the Time Stamp
    mySerial.print(lastTime);
    mySerial.print(",");
    mySerial.print("Heading(Yaw):");				//To read out the Heading (Yaw)
    mySerial.print(float(myEulerData.h) / 16.00);		//Convert to degrees
    mySerial.print(",");
    mySerial.print("Roll:");					//To read out the Roll
    mySerial.print(float(myEulerData.r) / 16.00);		//Convert to degrees
    mySerial.print(",");
    mySerial.print("Pitch:");				//To read out the Pitch
    mySerial.println(float(myEulerData.p) / 16.00);		//Convert to degrees
#else
    Serial.print("TimeStamp:");				//To read out the Time Stamp
    Serial.print(lastTime);
    Serial.print(",");
    Serial.print("Heading(Yaw):");				//To read out the Heading (Yaw)
    Serial.print(float(myEulerData.h) / 16.00);		//Convert to degrees
    Serial.print(",");
    Serial.print("Roll:");					//To read out the Roll
    Serial.print(float(myEulerData.r) / 16.00);		//Convert to degrees
    Serial.print(",");
    Serial.print("Pitch:");				//To read out the Pitch
    Serial.println(float(myEulerData.p) / 16.00);		//Convert to degrees
#endif    
  }
}

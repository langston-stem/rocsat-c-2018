// POST Pressure and Temperature test code
// Test to see if Pressure and Temperature sensor is working

// Needed Libraries
   #include <Wire.h>
   #include "bmp.h"

// Global variables
// variables for temperature and pressure
   short tempur;
   float tempurC;
   float tempurF;
   long presur;
   float presurhPa;
   float presurPSI;
   
void setup(){
// Start Serial communication.
   Serial.begin(9600);
   Serial.println("POST Pressure/Temperature code started...");

// Start I2C Communication
   Serial.println("START WIRE");
   Wire.begin();
  
// Calibrate the pressure and temperature sensor
   Bmp::bmp085Calibration();
  
}

void loop(){
// Read Temperature (C)
// Output is interger 
// Divide by resolution in data sheet 0.1 C
   tempur = Bmp::bmp085GetTemperature(Bmp::bmp085ReadUT()); 
   float tempurC = tempur/10.0;
   // Convert to F
   float tempurF = ((9.f/5)*tempurC+32);  
   
// Read Pressure (hPa) 
// Output is interger 
// Divide by resolution in data sheet 0.01 hPa
   long presur = Bmp::bmp085GetPressure(Bmp::bmp085ReadUP()); 
   float presurhPa = presur/100.0;
   // Convert to psi from 0.01 hPa
   float presurPSI = (presurhPa/(68.9475));
   
// Print results
   Serial.print("Temp(C): ");
   Serial.print(tempurC,1);
   Serial.print("\t Temp(F): ");
   Serial.print(tempurF,1);
   Serial.print("\t Pressure(hPa): ");
   Serial.print(presurhPa);
   Serial.print("\t Pressure(psi) ");
   Serial.println(presurPSI,3);
   
// delay for readability
   delay(500);
  }


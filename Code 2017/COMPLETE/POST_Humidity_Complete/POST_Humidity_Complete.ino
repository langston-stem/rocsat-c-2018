// POST Humidity test code
// Test to see if humidity sensor is working
// Will use temperature sensor as well
 
// Needed Libraries
   #include <Wire.h>
   #include "bmp.h"

// Global variables
// Humidity sensor on Analog pin A8
   const int HUM_SENS = A8; 
   int humid_raw;
   float humid_volt;
   float sensorRH;
   float trueRH;
   
// Temperature variables
   short tempur;
   float tempurC;

void setup(){
// Start Serial communication.
   Serial.begin(9600);  
   Serial.println("POST Humidity Sensor code started...");  

// Start I2C Communication
   Serial.println("START WIRE");
   Wire.begin();
   
// calibrate the pressure and temperature sensor
   bmp085Calibration();
  
// Sets up the pins as inputs 
   pinMode(HUM_SENS, INPUT);    
  
}

void loop(){  
// Read Temperature
   tempur = bmp085GetTemperature(bmp085ReadUT()); 
   tempurC = tempur/10.0;  

// Read Humidity and convert to voltage
   humid_raw = analogRead(HUM_SENS);
   humid_volt = humid_raw * 0.0049; 
   
// Convert to RH using temperature and equation from data sheet  
   sensorRH = ((humid_volt / 5.0) - 0.16) / 0.0062;
   trueRH = (sensorRH) / (1.0546 - 0.00216 * tempurC);
 
// Print the results   
   Serial.print("SensorRH ");
   Serial.print(sensorRH);
   Serial.print("\t TrueRH ");
   Serial.println(trueRH);
   
   delay(100); 

}


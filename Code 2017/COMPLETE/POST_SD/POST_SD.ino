/*
  POST_SD
*/

// The included arduino libraries
#include <SPI.h>
#include <SD.h>

// Global Variables //
boolean groundMode = 0;
const int PROG = 47;

// The included space shield libraries
#include "SDCard.h"

void setup() {

  //Begin serial communication
  Serial.begin(9600);
  Serial.println("Start SD POST code");

  //Initialize SPI
  SPI.begin();

  //Sets up the SD memory

  delay(1000);

  SDCardInit();
}

void loop () {
  //This will be the buffer we use to write to the SD
  dataString = String(digitalRead(PROG));
  
  if(writeDataToSD()){
    Serial.println("\nPrinted header to SD");
    Serial.println("Printed status of PROG pin (pull-data jumper) to SD");
  }
  else Serial.println("Error: data not written to SD");
  Serial.println("\nEnd of POST_SD");
  while (true);
}

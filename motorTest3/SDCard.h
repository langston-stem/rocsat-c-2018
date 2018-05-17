#ifndef _SDCard_h_
#define _SDCard_h_

const int groundMode = 0;

// Controls for the data logging system //
const int LOG_INTERVAL = 0;   // milli seconds between entries

// Time keeper
uint32_t timeStamp = 0;     // The time stamp used when recording data points

const int chipSelect = 53;  // This is set to 53 for the RockOn Shield

// Variable for file name
char logFileName[16];

int ledState = 0;

//Character strings for writing data to memory //
String dataString = ""; //holds the entire data string for each read cycle
String sensorNames = "PROG PIN STATUS";

//--->FUNCTION is called when the system starts or after a power reset to enable recording dat to the SD card.
boolean SDCardInit() {
  Serial.println("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    //delay(100);
    // Can't do anything more.  Try another time....
    return 0;
  }
  
  // The Card is present so find an unused file name
  Serial.println("searching for an unused file name... ");
  // Start with LOG0.CSV and count up until an unused file name is found.
  for (long i = 0; i < 1000; i++)
  {
    sprintf(logFileName, "LOG%d.CSV", i);
    if (!SD.exists(logFileName))
    {
      break;
    }
  }

  Serial.print("The Log filename is:  ");
  Serial.println(logFileName);

  File dataFile = SD.open(logFileName, FILE_WRITE);

  // Write the header including sensor names to the newly opened file
  dataString = sensorNames;

  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println("SD Card initialized and data written.");
    return (1);                          // Able to write to SD card
  }
  else
  {
    Serial.println("SD card present but unable to write to file");
    sprintf(logFileName, "LOG0.CSV");           // Clear out the file name to force the program to find a unused file name
    return (0);                           // Unable to write to SD card
  }
}

//---->FUNCTION
/* This function writes data to the SD card.  It returns a 1 (TRUE), 
   if it successfully wrote to the SD Card and if there was an error 
   it returns a 0 (FALSE).
*/
boolean writeDataToSD() {
  File dataFile = SD.open(logFileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
    return (1);                                             // Return a 1 since the write was successful LED Blink
  }

  // if the file isn't open, notify that there was an error and re-initialize  the card
  else {
    if (groundMode) Serial.println();
    if (groundMode) Serial.print("error writing to file: ");
    if (groundMode) Serial.println(logFileName);
    dataFile.close();
    delay(100);
    if (groundMode) Serial.println("Re - Initializing SD card...");
    return (SDCardInit());          //Return a 0 if successful to disable LED Blink
  }
}

#endif


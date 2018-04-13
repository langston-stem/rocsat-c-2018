// POST Accelerometer test 
// Test to see if your accelorometer is working only

// Global Variables
// Low accelerometer variables
   const int ACCL_X = A3;  
   const int ACCL_Y = A4;  
   const int ACCL_Z = A5;
   int LX;
   int LY;
   int LZ;
   float LXmV;
   float LYmV;
   float LZmV;
   
// Medium accelerometer variables
   const int ACCM_X = A0;  
   const int ACCM_Y = A1;  
   const int ACCM_Z = A2;
   int MX;
   int MY;
   int MZ;
   float MXmV;
   float MYmV;
   float MZmV;
   
// High accelerometer variables
   const int ACCH_Z = A6;
   int HZ;
   float HZmV;
   
 void setup() {
// initialize serial communication at 9600 bits per second:
   Serial.begin(9600);
//Sets up the pins as inputs
   pinMode(ACCL_X,INPUT);
   pinMode(ACCL_Y,INPUT);
   pinMode(ACCL_Z,INPUT);
   pinMode(ACCM_X,INPUT);
   pinMode(ACCM_Y,INPUT);
   pinMode(ACCM_Z,INPUT);
   pinMode(ACCH_Z,INPUT);
  }

 void loop() {
//Read the accelerometer. (output is a number between 0-1023)
   LX = analogRead(ACCL_X);
   LY = analogRead(ACCL_Y);
   LZ = analogRead(ACCL_Z);
   MX = analogRead(ACCM_X);
   MY = analogRead(ACCM_Y);
   MZ = analogRead(ACCM_Z);
   HZ = analogRead(ACCH_Z);

//Convert readings into millivolts
   LXmV = LX * 4.9;
   LYmV = LY * 4.9;
   LZmV = LZ * 4.9;
   MXmV = MX * 4.9;
   MYmV = MY * 4.9;
   MZmV = MZ * 4.9;
   HZmV = HZ * 4.9;

// print out the X Accel values 
   Serial.print("X ");
   Serial.print(LX);
   Serial.print("\t");
   Serial.print(LXmV,0);
   Serial.print("\t");
   Serial.print(MX);
   Serial.print("\t");
   Serial.print(MXmV,0);
   Serial.print("\t");
   
// print out the Y Accel values    
   Serial.print("Y ");
   Serial.print(LY);
   Serial.print("\t");
   Serial.print(LYmV,0);
   Serial.print("\t");
   Serial.print(MY);
   Serial.print("\t");
   Serial.print(MYmV,0);
   Serial.print("\t");
   
// print out the Z Accel values    
   Serial.print("Z ");
   Serial.print(LZ);
   Serial.print("\t");
   Serial.print(LZmV,0);
   Serial.print("\t");
   Serial.print(MZ);
   Serial.print("\t");
   Serial.print(MZmV,0);
   Serial.print("\t");   
   Serial.print(HZ);
   Serial.print("\t");
   Serial.println(HZmV,0);
   
// delay in between reads for stability
   delay(100);        
 }







//  POST Accelerometer test
// Test to see if your accelorometer is working

// Global Variables
// Low accelerometer variables
   const int ACCL_LX  =  A3;
   const int ACCL_LY  =  A4;
   const int ACCL_LZ = A5;
   int LX;
   int LY;
   int LZ;
   int LXmV;
   int LYmV;
   int LZmV;
   float LXaccel, LYaccel, LZaccel;
   float MXaccel, MYaccel, MZaccel;
   float HZaccel;

// Medium accelerometer variables
   const int ACCL_MX  =  A0;
   const int ACCL_MY  =  A1;
   const int ACCL_MZ = A2;
   int MX;
   int MY;
   int MZ;
   float MXmV;
   float MYmV;
   float MZmV;
   
   
// High accelerometer variables
   const int ACCL_HZ = A6;
   int HZ;
   float HZmV;
void setup() {
  // initialize serial communication at 9600 bits
     Serial.begin(9600);
  // sets up the pins as inputs
      pinMode(ACCL_LX, INPUT);
      pinMode(ACCL_LY, INPUT);
      pinMode(ACCL_LZ, INPUT);
      pinMode(ACCL_MX, INPUT);
      pinMode(ACCL_MY, INPUT);
      pinMode(ACCL_MZ, INPUT);
      pinMode(ACCL_HZ, INPUT);
}

void loop() {
  // Read the acceleramerte. (output is a number:
       LX  =analogRead(ACCL_LX);
       LY  =analogRead(ACCL_LY);
       LZ  =analogRead(ACCL_LZ);
       MX  =analogRead(ACCL_MX);
       MY  =analogRead(ACCL_MY);
       MZ  =analogRead(ACCL_MZ);
       HZ  =analogRead(ACCL_HZ);

  // Convert readings into millivolts
     LXmV = LX * 5000L / 1023;
     LYmV = LY * 5000L / 1023;
     LZmV = LZ * 5000L / 1023;
     MXmV = MX * 5000L / 1023;
     MYmV = MY * 5000L / 1023;
     MZmV = MZ * 5000L / 1023;
     HZmV = HZ * 5000L / 1023;

    // convert mV to G (acceleration) Normail mV is 1650 however changed to 1617 due to experimental data
     LXaccel = (LXmV - 1617) / 300.0;
     LYaccel = (LYmV - 1617) / 300.0;
     LZaccel = (LZmV - 1617) / 300.0;
     MXaccel = (MXmV - 1617) / 57.0;
     MYaccel = (MYmV - 1617) / 57.0;
     MZaccel = (MZmV - 1617) / 57.0;
     HZaccel = (HZmV - 2443) /  27.0 ;

  // print out the X Accel values
     Serial.print ("X ");
     Serial.print (LXaccel, 2);
     Serial.print ("\t");
     Serial.print ("MX ");
     Serial.print (MXaccel, 2);
     Serial.print ("\t");
     Serial.print ("Y ");
     Serial.print(LYaccel, 2);
     Serial.print ("\t");
     Serial.print ("MY ");
     Serial.print (MYaccel, 2);
     Serial.print ("\t");
     Serial.print ("Z ");
     Serial.print(LZaccel, 2);
     Serial.print ("\t");
     Serial.print ("MZ ");
     Serial.print (MZaccel, 2);
     Serial.print ("\t");
     Serial.print ("HZ ");
     Serial.print (HZaccel, 2);
     Serial.print ("\t");
     Serial.print("\n");
     Serial.print (HZmV, 2);
}


// POST Accelerometer test 
// Test to see if your accelorometer is working only

// Global Variables
// Low accelerometer variables





   
// Medium accelerometer variables





   
// High accelerometer variables



   
 void setup() {
// initialize serial communication at 9600 bits per second:


//Sets up the pins as inputs






  }

 void loop() {
//Read the accelerometer. (output is a number between 0-1023)







//Convert readings into millivolts









// print out the X Accel values 







   
// print out the Y Accel values    







   
// print out the Z Accel values    






   
// delay in between reads for stability
   delay(100);        
 }





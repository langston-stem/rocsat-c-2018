// POST Geiger Counter (GC) test code
// Test to see if GC is working

// Global variables
// GC output is connected to pin 18

   
// Attach 5th interrupt of the Arduino Mega.


// Declared volatile because two threads of execution are using it.
// Value is initially set to zero because there are no counts.   

   
// gc_cnt to store the number of counts
// from gc_counts no counts are missed 
// while writing to buffer 


// Function for interrupt 
   // gc_counts is increased by 1 
   // every time function called.
   // void loop will reset when written memory 



void setup(){
// Sets up communication with computer

   
// Sets up gc_pin as input

  
// Sets up the interrupt to trigger for a rising edge
// gc_pin (18) corresponds to the 5th interrupt gc_intnumber  
// gc_interrupt is the function we want to call once we detect an interrupt

}

void loop(){
// delay 5 secconds




   
// prints results on screen
 
 
   
}


// POST Geiger Counter (GC) test code
// Test to see if GC is working

// Global variables
// GC output is connected to pin 18
   const unsigned int gc_pin = 18;
   
// Attach 5th interrupt of the Arduino Mega.
   const unsigned int gc_intnumber = 5;

// Declared volatile because two threads of execution are using it.
// Value is initially set to zero because there are no counts.   
   volatile unsigned int gc_counts = 0;
   
// gc_cnt to store the number of counts
// from gc_counts no counts are missed 
// while writing to buffer 
   unsigned int gc_cnt;

// Function for interrupt 
   // gc_counts is increased by 1 
   // every time function called.
   // void loop will reset when written memory 
   void gc_interrupt(){ 
     gc_counts++;
   }   

void setup(){
// Sets up communication with computer
   Serial.begin(9600);
   
// Sets up gc_pin as input
   pinMode(gc_pin, INPUT);
  
// Sets up the interrupt to trigger for a rising edge
// gc_pin (18) corresponds to the 5th interrupt gc_intnumber  
// gc_interrupt is the function we want to call once we detect an interrupt
   attachInterrupt(gc_intnumber, gc_interrupt, RISING);
}

void loop(){
// delay 5 secconds
   Serial.println("5 second data window started...");
   delay(5000);
   
   gc_cnt = gc_counts;
   gc_counts = 0;
   
// prints results on screen
   Serial.println(gc_cnt);
   Serial.println("Geiger counter reset"); 
   Serial.println(" ");   
}


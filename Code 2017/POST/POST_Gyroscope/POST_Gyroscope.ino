// POST Gyroscope test code
// Test to see if Gyroscope is working

// Needed Libraries



// Global variables
// variables for Gryo X,Y,Z axes




void setup(){
// Start Serial communication.
 


// Start I2C Communication



// Start Gyro


// Starts the gyro
 

// Prints addres of gyro

  
}

void loop(){
  
// Read Gyro
// The values after the Gyro::read_() command are initializing the output
// Calculate the initialization values by placing the shield on a flat
// surface and not moving or shaking it. Those values should be added
// here in the code and must be a whole number





// Print the output rates to the terminal, seperated by a TAB character.
// Right now we do not have a way to calibrate the data collected but this 
// shows that tilting the board results in a change in numbers displayed






}



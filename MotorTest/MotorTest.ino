
const unsigned int motorA_pin = 2;
const unsigned int motorA_int = 0;
volatile unsigned int motorA_counts = 0;

void motorA_int_handler() {
  motorA_counts++;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(motorA_pin, INPUT);
  attachInterrupt(motorA_int, motorA_int_handler, RISING);
  

}

void loop() {
  // put your main code here, to run repeatedly:
unsigned int count, rpm;
unsigned long t = millis();

//Serial.println("It has started");
delay(1000);
count = motorA_counts;
rpm = count/150*60;
motorA_counts = 0;
Serial.print("Your count is ");
Serial.print(count);
Serial.print(" the rpm is ");
Serial.println(rpm);

}

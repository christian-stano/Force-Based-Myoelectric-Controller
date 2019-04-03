// Include the Servo library
#include <Servo.h> 

// Declare the Servo pin
int servoPin = A0;
// Create a servo object
Servo Servo1;

void setup() {
  // We need to attach the servo to the used pin number
  Servo1.attach(servoPin);
}

void loop(){
  // Make servo go to 0 degrees
  Servo1.writeMicroseconds(750);
  delay(2000);
  Servo1.writeMicroseconds(2250);
  delay(1000);
 // Servo1.writeMicroseconds(1500);
  //delay(1000);
  //Servo1.writeMicroseconds(0);
 //delay(1000);
}

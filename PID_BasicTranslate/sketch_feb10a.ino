/*  AnalogReadSerial  Reads an analog input on pin 0, prints the result to the serial monitor.  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground. This example code is in the public domain. */
#include <PID_v1.h>

// Define variable for use in the PID 
double setpoint, sensorValue, microOut; 

PID myPID(&sensorValue, &microOut, &setpoint, 10, 0, 0, DIRECT); 


// the setup routine runs once when you press reset:
void setup() {
  sensorValue = analogRead(A0);
  setpoint = 450; 
  myPID.SetMode(AUTOMATIC);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  sensorValue = analogRead(A0);
  //int sensorValue2 = analogRead (A1);

  myPID.Compute();
  analogWrite(A1, microOut); 

  // print out the value you read:
    Serial.print(sensorValue);
   // Serial.print(" ");   
   // Serial.print(",");              //seperator
   // Serial.print(" ");   
   // Serial.println(sensorValue2);          //the second variable for plottingincluding line break
    Serial.print(" ");   
    Serial.print(",");              //seperator
    Serial.print(" ");   
    Serial.println(microOut);
    
  delay(1);        // delay in between reads for stability
}



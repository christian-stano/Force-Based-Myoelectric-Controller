/*  AnalogReadSerial  Reads an analog input on pin 0, prints the result to the serial monitor.  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground. This example code is in the public domain. */
#include <PID_v1.h>

// Define variable for use in the PID 
double setpoint, sensorValue, microOut; 
double comparator, return_compare; 

// set PID with input, outputs, and coefficients 
PID myPID(&sensorValue, &microOut, &setpoint, 0.5, 1, 20, DIRECT); 


// the setup routine runs once when you press reset:
void setup() {
  sensorValue = diff_output; // input is differential output 
  setpoint = 0; // 0 volts as output to servo when contraction held for period of time 

   // Activated PID under automatic operation
  myPID.SetMode(AUTOMATIC);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:or until ended by power disconnect 
void loop() {

  comparator = sensorValue - setpoint; 
  return_compare = 0.02*(max_contract - setpoint); 

  // OR IF THIS IS KNOWN 

  return_compare = 0.02* Noise_level; 

  //if the sensed contraction is less than 2% of the max_contraction possible 
  if (comparator <= return_compare) {
      change_in_position = current_location - relaxed_position; 
      micro_out = voltage_to_Change_in_position; 
  }
  else 
{
  //compute defined PID with parameters 
  myPID.Compute();

  //Write output to servo point of communication 
  analogWrite(A2, microOut); 

  // print out the values in and out if wanted for manipulation 
   // Serial.print(sensorValue);
   // Serial.print(" ");   
   // Serial.print(",");              //seperator
   // Serial.print(" ");    
   // Serial.println(microOut);
}
  delay(1);        // delay in between reads for stability
}



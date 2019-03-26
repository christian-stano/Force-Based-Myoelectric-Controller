/*  AnalogReadSerial  Reads an analog input on pin 0, prints the result to the serial monitor.  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground. This example code is in the public domain. */
// Define variable for use in the PID 

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
 
  Serial.begin(9600);
   Serial.println("LABEL,Time, MuscleA0,MuscleA1");
   Serial.println("RESETTIMER"); //resets timer to 0
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  int sensorValue2 = analogRead (A1);

  //double outputfinal= analogRead(A2); 
  // print out the value you read:
    Serial.print("DATA, TIME,"); 
    Serial.print(sensorValue);
    Serial.print(" ");
    Serial.print(",");
    Serial.print(" ");
    Serial.println(sensorValue2);   //the second variable for plottingincluding line break

    
  delay(10);        // delay in between reads for stability
}

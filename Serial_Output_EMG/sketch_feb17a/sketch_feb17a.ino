/* The following program allows for the serial observation of Raw EMG signal inputs from a Myoware 
 *  Microcontroller to a Teensy v3.6 development board. The PLX-DAQ program can be downloaded separately 
 *  to allow for the recording of this data in an excel file. To record in the program, simple run this 
 *  program, open the PLX-DAQ spreadsheet and connect to the correct USB port. For real-time visulaization 
 *  of the dataset, go to Tools -> Serial Plotter and see the Extensor Data in Blue and the Flexor Data in 
 *  Red. 
 */
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Initialize Column Output Headers for use of the PLX-DAQ 
   Serial.println("LABEL,Time, MuscleA0,MuscleA1");
   Serial.println("RESETTIMER"); //resets timer to 0 for the PLX-DAQ Outputs 
}


void loop() {
  // read the input on analog pin 0 from the teensy board (Extensor):
  int sensorValue = analogRead(A0);
  // read the input on analog pin 1 from the teensy board (Flexor):
  int sensorValue2 = analogRead (A1);

  // print out values read from 
  
    Serial.print("DATA, TIME,"); // Initialize data output to PLX-DAW with time stamp 
    Serial.print(sensorValue);// Print Extensor Data Serially 
    Serial.print(" "); // Separate outputs so they are viewed as distinct value for plotting 
    Serial.print(",");
    Serial.print(" ");
   Serial.println(sensorValue2);   //Print Flexor Data Serially with line break to move to next data port. 

    
  delay(10); // delay in between reads to allow for data visualization 
}

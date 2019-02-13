//Number of data points to be read in each window
const int buffersize = 60;

uint8_t bufferVals[buffersize,2]; //store value of flexor digitorum and extensor digitorum input
uint8_t index = 0; //current index of array

//set threshold for action to ~10% of max contraction (1024)
uint8_t threshold = 100; 

void setup() {
  Serial.begin(9600);
}

void loop() {
  while (index < buffersize) {
    bufferVals[index,0] = analogRead(A0); //store flexor digitorum value
    bufferVals[index,1] = analogRead(A1); //store extensor digitorum value
    index++;
  }
  index = 0; 

  
  
  delay(1); //delay between reads for stability
}

#include <RunningAverage.h>

//constructor of running average with memory allocation of 10
RunningAverage myRAFlexor(4);
RunningAverage myRAExtensor(4);
//track how many samples have entered buffer 
int buffer_sample_size = 0;

//initial difference in EMG channels = 0
int prevDiff = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  myRAFlexor.clear(); //explicitly clears running average buffer
  myRAExtensor.clear();
}

// the loop routine runs over and over again forever:
void loop() {
  
  /* read the input on analog pin 0 (flexor digitorum/top of arm) and pin 1 (extensor digitorum/bottom of arm):
   * analog values range from 0 - 1023
   */
  int flexorAnalogVal = analogRead(A0);
  int extensorAnalogVal = analogRead(A1);
  myRAFlexor.addValue(flexorAnalogVal);
  myRAExtensor.addValue(extensorAnalogVal);
  buffer_sample_size++;

  if (buffer_sample_size == 5)
  {
    int flexorWindowedVal = myRAFlexor.getAverage();
    int extensorWindowedVal = myRAExtensor.getAverage();
    // print out the value you read:
    Serial.println(flexorWindowedVal);
    Serial.println(extensorWindowedVal);

    //differential comparator
    int differentialcomparator = differential_comparator(flexorWindowedVal, extensorWindowedVal, prevDiff);
    
    buffer_sample_size = 0;
    myRAFlexor.clear();
    myRAExtensor.clear();
  }

  // print out the value you read:
  Serial.println(flexorAnalogVal);
  Serial.println(extensorAnalogVal);
}

// first part of EMG differential algorithm that subtracts the value of the extensor from the flexor
// uses threshold of 10% difference from previous differential value (prevDiff) to calculate if above threshold
// returns new difference if difference > threshold, else returns prevDiff 
int differential_comparator(int flexorAnalogVal, int extensorAnalogVal, int prevDiff) {
  int curDiff = flexorAnalogVal - extensorAnalogVal;

  int percentDifference = (abs(curDiff)-abs(prevDiff))/abs(prevDiff) * 100;
  if (percentDifference < 10) {
    return prevDiff;
  } else {
    return curDiff;
  }
}

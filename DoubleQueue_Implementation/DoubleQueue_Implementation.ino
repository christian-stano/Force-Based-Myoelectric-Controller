#include <QueueArray.h>

//instantiate data buffers to be used to collect, process, and flush in sequence
QueueArray <int> dataBuffer1;
QueueArray <int> dataBuffer2;

//initial EMG channel difference
int prevDiff = 0;

//declare and allocate space for incoming analog vals
uint8_t flexorAnalogVal = 0;
uint8_t extensorAnalogVal = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

}

void loop() {
   /* read the input on analog pin 0 (flexor digitorum/top of arm) and pin 1 (extensor digitorum/bottom of arm):
   * analog values range from 0 - 1023
   */
  while (dataBuffer1.count() < 60) { //collecting into buffer 1
    flexorAnalogVal = analogRead(A0);
    extensorAnalogVal = analogRead(A1);
    dataBuffer1.enqueue(differential_comparator(flexorAnalogVal,extensorAnalogVal,prevDiff)); //Enqueue EMG Differential values
  }
  

}

// first part of EMG differential algorithm that subtracts the value of the extensor from the flexor
// uses threshold of 10% difference from previous differential value (prevDiff) to calculate if above threshold
// returns new difference if difference > threshold, else returns prevDiff 
int differential_comparator(uint8_t flexorAnalogVal, uint8_t extensorAnalogVal, int prevDiff) {
  int curDiff = flexorAnalogVal - extensorAnalogVal;

  int percentDifference = (abs(curDiff)-abs(prevDiff))/abs(prevDiff) * 100;
  if (percentDifference < 10) {
    return prevDiff;
  } else {
    return curDiff;
  }
}

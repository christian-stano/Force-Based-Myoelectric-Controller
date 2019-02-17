#include "Arduino.h"
#include "MyoWare.h"

static const unsigned int sampleTime = 1;
static const double adcRef = 5.00; //adc reference for conversion from analog to mV
static const unsigned int adcRes = 1023; //adc resolution for conversion from analog to mV
static const double adcConv = adcRef/adcRes; //scaling factor to convert adc to mV
sampleCounter = 0; //determines index of array to populate with data point
b1 = true; // b1-b4 indicates what section of buffer array is new and old data to be populated or flush
b2 = false;
b3 = false;
b4 = false;

//Constructor
MyoWare::MyoWare(uint8_t EMG_PIN) {
    pinMode(EMG_PIN, INPUT);
    EMG_PIN = _EMG_PIN;
}

//Samples the EMG every 1 ms
void MyoWare::sampling() {
    emg = analogRead(_EMG_PIN);
    sampleOk = true; //indicates that a new sample is able to be processed
    bufferManager();
}

//Manages insertion of each EMG data point into correct position of 200 element buffer
void MyoWare::bufferManager() {
    if (b1) { //insert in 0-49
        bufferArray[sampleCounter] = emg;
        sampleCounter++;
    } else if (b2) { //insert in 50-99
        bufferArray[sampleCounter+50] = emg;
        sampleCounter++;
    } else if (b3) { //insert in 100-149
        bufferArray[sampleCounter+100] = emg;
        sampleCounter++;
    } else { //insert in 150-199
        bufferArray[sampleCounter+150] = emg;
        sampleCounter++;
    }

    //cycles 50 ms populating partition of buffer array every 50 ms
    if (sampleCounter == 49) {
        sampleCounter = 0;
        if (b1) {
            b1 = false;
            b2 = true;
        } else if (b2) {
            b2 = false;
            b3 = true;
        } else if (b3) {
            b3 = false;
            b4 = true;
        } else {
            b4 = false;
            b1 = true;
        }
    }
}

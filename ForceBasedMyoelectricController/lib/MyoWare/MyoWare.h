#ifndef MyoWare_h
#define MyoWare_h

#include "Arduino.h"

class MyoWare {
public:
    MyoWare(uint8_t EMG_PIN);
    void sampling();
    bool activation();
    // void calibration(); implement in next iteration

private:
    void meanCalc();
    // double movingAverage();
    // void mvcCalc(unsigned int mvcSamples);
    void bufferManager();
    void MAV();
    uint8_t _EMG_PIN;
    volatile unsigned int emg;
    static int bufferArray[200];
    bool b1, b2, b3, b4;
    int sampleCounter;
    double emgMean;
    double emgSum;
    double emgMVC;
    bool sampleOk, isActive;
};

#endif

#ifndef Processing_h
#define Processing_h

#include "Arduino.h"
#include "MyoControl.h"

class Processing {
public:

private:
    void bufferManager();
    static int bufferArray[200];
    bool b1, b2, b3, b4;
    int sampleCounter;
};

#endif

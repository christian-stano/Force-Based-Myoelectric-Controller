/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * MyoControl is distributed under a GPL 3.0 license
 */

#ifndef MyoControl_h
#define MyoControl_h

#include <Arduino.h>

class MyoControl {
    public:
        MyoControl(int emg_pin);
        void calibrationSampling();
        double sampling();
        void calibration();
        double slopeCalc(int muscle, double emgMVC);
        double interceptCalc(int muscle);
        // void activation();
        // void printSamples();
    private:
        // void blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int bTime);
        void meanCalc(unsigned int meanSamples);
        // double movAv();
        // void mvcCalc(unsigned int mvcSamples);
        int emgpin;
        volatile unsigned int emg;
        double emgMean;
        double emgAvg;
        double slope;
        // unsigned int sampleCounter;
        double emgMVC;
        bool sampleOk, isActive;
        double emg_u_prev;
        double emg_f_prev;
        // double bufferArray[200];
};

#endif

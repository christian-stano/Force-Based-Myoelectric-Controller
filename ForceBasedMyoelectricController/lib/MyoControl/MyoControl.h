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
        void sampling();
        void calibration();
        void activation();
    private:
        // void blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int bTime);
        void meanCalc(unsigned int meanSamples);
        double movAv();
        void mvcCalc(unsigned int mvcSamples);
        int emgpin;
        volatile unsigned int emg;
        double emgMean;
        double emgMvc;
        bool sampleOk, isActive;
};

#endif

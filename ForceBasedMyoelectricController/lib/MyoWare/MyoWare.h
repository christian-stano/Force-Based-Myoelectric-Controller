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
        MyoControl(uint8_t emg_pin);
        void sampling();
        void calibration();
        bool activation();
    private:
        void blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int bTime);
        void meanCalc(unsigned int meanSamples);
        double movAv();
        void mvcCalc(unsigned int mvcSamples);
        uint8_t _emg_pin;
        volatile unsigned int emg;
        double emgMean;
        double emgMvc;
        bool sampleOk, isActive;
};

#endif

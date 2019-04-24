/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * MyoControl is distributed under a GPL 3.0 license
 */

 /*
 * Force Based Controller for Myoelectric Prostheses
 * Authors: Christian Stano, Allyson King
 * Email: cstano29@gmail.com
 * This code and the associated libraries were developed by students at Vanderbilt University
 under the guidance of Emily Gracyzk, Ph.D of Case Western Reserve University
 * This library provides functionality for interfacing with the MyoWare sensors for calibration and sampling
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
    private:
        void meanCalc(unsigned int meanSamples);
        int emgpin;
        volatile unsigned int emg;
        double emgMean;
        double slope;
        double emgMVC;
        bool sampleOk, isActive;
        double emg_u_prev;
        double emg_f_prev;
};

#endif

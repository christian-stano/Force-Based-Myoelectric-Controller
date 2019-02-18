/*
 * Copyright 2019 Autofabricantes.
 * Author: Alvaro Villoslada (Alvipe).
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * MyoControl is distributed under a GPL 3.0 license.
 */

#include "Arduino.h"
#include "MyoControl.h"

static const unsigned int sampleTime = 1;
static const double adcRef = 5.00;
static const unsigned int adcRes = 1023;
static const double adcConv = adcRef/adcRes;

MyoControl::MyoControl(uint8_t emg_pin) {
    pinMode(emg_pin, INPUT);
    emg_pin = _emg_pin;
}

/* blinkLED blinks a led "repeat" times with a "bTime" interval between on and off */
void MyoControl::blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int bTime) {
    pinMode(ledPin, OUTPUT);
    unsigned int i;
    for(i=0;i<repeat;i++) {
        digitalWrite(ledPin,HIGH);
        delay(bTime);
        digitalWrite(ledPin,LOW);
        delay(bTime);
    }
}

/* sampling reads the ADC every 1 ms with the MsTimer2 interrupt. */
void MyoControl::sampling() {
    emg = analogRead(_emg_pin);
    sampleOk = true; // sampleOk indicates that a new sample is ready to be processed
}

/* meanCalc computes the mean value of the EMG signal during a period of
meanSamples. The mean value of the signal is its baseline. This value is
subtracted from the measured EMG signals to have a baseline of 0 V. */
void MyoControl::meanCalc(unsigned int meanSamples)
{
    unsigned int i = 0;
    while(i < meanSamples)
    {
        delayMicroseconds(50);
        if(sampleOk)
        {
            sampleOk = false;
            i++;
            emgMean = emgMean + emg*adcConv;
        }
    }
    i = 0;
    emgMean = emgMean/meanSamples;
}

/* movAv computes the running moving average of the EMG signal. First, the
baseline of the signal is lowered to 0 V, to be able to rectify it. The running
moving average rectifies and smooths the signal (acts as a low pass filter),
returning the amplitude of the signal. */
double MyoControl::movAv() {
    double emgZero = 0.00, emgMovav = 0.00;
    emgZero = emg*adcConv - emgMean; // Signal with 0 V baseline
    emgMovav = emgMovav*0.99 + abs(emgZero)*0.01; // Rectified and smoothed signal
    return emgMovav;
}

/* mvcCalc computes the maximum voluntary contraction, the maximum force the
user is able to exert. This value is used to compute the activation threshold */
void MyoControl::mvcCalc(unsigned int mvcSamples) {
    double emgMovav = 0.00;
    unsigned int i = 0;
    while(i < mvcSamples) {
        delayMicroseconds(50);
        if(sampleOk) {
            sampleOk = false;
            i++;
            emgMovav = movAv();
            if(emgMovav > emgMvc) {
                emgMvc = emgMovav;
            }
        }
    }
}

void MyoControl::calibration() {
    /* System calibration */
    /* Calibration step #1: calculate the baseline of the signal during 10 s */
    blinkLED(13,1,500); // LED blinks once to indicate calibration step #1 start
    meanCalc(10000);
    blinkLED(13,1,500); // LED blinks once to indicate calibration step #1 end
    delay(1000);
    /* Calibration step #2: calculate the maximum voluntary contraction during 5 s*/
    blinkLED(13,2,500); // LED blinks twice to indicate calibration step #2 start
    mvcCalc(5000);
    blinkLED(13,2,500); // LED bliks twice to indicate calibration step #2 end
    delay(1000);
}

bool MyoControl::activation() {
    delayMicroseconds(50);
    if(sampleOk) {
        sampleOk = false;
        double emgMovav = movAv(); // Gets the amplitude of the measured EMG signal
        /* If the amplitude of the EMG signal is greater than the activation threshold
        (a 35% of the MVC), there is a muscle activation. */
        if(emgMovav > 0.35*emgMvc) {
            isActive = true;
        }
        /* If the amplitude of the EMG signal is below the activation threshold,
        there is no muscle activation. */
        else {
            isActive = false;
        }
    }
    return isActive;
}

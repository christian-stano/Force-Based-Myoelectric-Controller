/*
 * Copyright 2019 Autofabricantes.
 * Author: Alvaro Villoslada (Alvipe).
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * MyoControl is distributed under a GPL 3.0 license.
 */

 /*
 * Force Based Controller for Myoelectric Prostheses
 * Authors: Christian Stano, Allyson King
 * Email: cstano29@gmail.com
 * This code and the associated libraries were developed by students at Vanderbilt University
 under the guidance of Emily Gracyzk, Ph.D of Case Western Reserve University
 * This library provides functionality for interfacing with the MyoWare sensors for calibration and sampling
 */

#include "Arduino.h"
#include "MyoControl.h"
#include "IntervalTimer.h"

static const unsigned int sampleTime = 1;
static const double adcRef = 5.00;
static const unsigned int adcRes = 1023;
static const double adcConv = adcRef/adcRes; //convert bits to voltage- not used in below code
//Tustin approximation/lowpass filter constants
static const double fc = 1; //low pass recursive Tustin approximation cutoff frequency (Hz)
static const double tau = 1/(2*PI*fc);
static const double alpha1 = (-0.001+2*tau)/(2*tau+0.001);
static const double alpha2 = 0.001/(2*tau+0.001);

/*
 * MyoControl object constructor
 * Creates a MyoControl object paired with the MyoWare sensor at emg_pin
 * Initializes emg_u_prev and emg_f_prev (needed values for Tustin approximation)
 to 0 and initializes object with following fields: emg, emgMean, slope,
 emgMVC, sampleOk, isActive
*/
MyoControl::MyoControl(int emg_pin) {
    emgpin = emg_pin;
    emg_u_prev = 0;
    emg_f_prev = 0;
}

/* sampling reads the ADC every 1 ms with the intervaltimer interrupt during
calibration
 * Called from interrupt function in main.ino every 1 ms
*/
void MyoControl::calibrationSampling() {
    emg = analogRead(emgpin);
    sampleOk = true; // sampleOk indicates that a new sample is ready to be processed
    noInterrupts(); // prevents interrupt overwriting most recent value before it is processed
}

/* Samples ADC every 1 ms, performs baseline rectification, Tustin approximation,
and returns to main.ino
 * Called from interrupt function in main.ino every 1 ms
*/
double MyoControl::sampling() {
    emg = analogRead(emgpin);

    //Baseline removal and Rectify
    double emgBaseline = emg-emgMean;
    double emgRectify = abs(emgBaseline);
    //Tustin approximation/smoothing
    double emgFilt = alpha1*emg_f_prev + alpha2*(emgRectify+emg_u_prev);
    emg_f_prev = emgFilt;
    emg_u_prev = emgRectify;

    return emgFilt;
}

/* meanCalc computes the mean value of the EMG signal during a period of
meanSamples. The mean value of the signal is its baseline. This value is
subtracted from the measured EMG signals to have a baseline of 0
 * Called from calibration() function below
*/
void MyoControl::meanCalc(unsigned int meanSamples)
{
    unsigned int i = 0;
    while(i < meanSamples)
    {
        delayMicroseconds(50);
        if(sampleOk) // true = sample available for processing
        {
            sampleOk = false;
            i++;
            emgMean = emgMean + emg; //keep running sum

        }
        interrupts(); // resume calibrationSampling interrupts
    }
    i = 0;
    emgMean = emgMean/meanSamples; //calculate mean over sampling period
    Serial.print("EMG Mean is: ");
    Serial.println(emgMean);
}

/*
 * slopeCalc function
 * Calculates the slope of the upper or lower bounds of the linear piecewise classifier
 * Input: muscle- 1 for extensor (positive) or 0 for flexor (negative)
 * Input: emgMVC- average MVC calculated during MVC calibration
 * Called from main.ino setup() function after MVC collection
*/
double MyoControl::slopeCalc(int muscle, double emgMVC) {
    slope = (muscle*90-muscle*75)/(emgMVC-(muscle*3));
    return slope;
}

/*
 * interceptCalc function
 * Calculates the intercept of the upper or lower bounds of the linear piecewise classifier
 * Input: muscle- 1 for extensor (positive) or 0 for flexor (negative)
 * Called from main.ino setup() function after MVC collection
*/
double MyoControl::interceptCalc(int muscle) {
    double intercept;
    intercept = muscle*75-slope*muscle*3;
    return intercept;
}

/*
 * calibration function
 * Calculation of baseline for use in baseline removal during processing
 * Called from main.ino setup() function
*/
void MyoControl::calibration() {
    /* System calibration */
    /* Calibration step #1: calculate the baseline of the signal during 10 s */
    Serial.println("Calibration: keep muscles relaxed for 10 s");
    meanCalc(10000); // 10 seconds of samples collected
    Serial.println("Relaxed calibration complete");
    delay(1000);
}

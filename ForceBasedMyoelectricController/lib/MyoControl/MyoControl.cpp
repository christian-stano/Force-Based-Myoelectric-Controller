/*
 * Copyright 2019 Autofabricantes.
 * Author: Alvaro Villoslada (Alvipe).
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * MyoControl is distributed under a GPL 3.0 license.
 */

#include "Arduino.h"
#include "MyoControl.h"
#include "IntervalTimer.h"

static const unsigned int sampleTime = 1;
static const double adcRef = 5.00;
static const unsigned int adcRes = 1023;
static const double adcConv = adcRef/adcRes; //convert bits to voltage
static const double fc = 1; //low pass recursive Tustin approximation cutoff frequency
static const double tau = 1/(2*PI*fc);
static const double alpha1 = (-0.001+2*tau)/(2*tau+0.001);
static const double alpha2 = 0.001/(2*tau+0.001);

MyoControl::MyoControl(int emg_pin) {
    emgpin = emg_pin;
    emg_u_prev = 0;
    emg_f_prev = 0;
    // sampleCounter = 0;
}

// /* blinkLED blinks a led "repeat" times with a "bTime" interval between on and off */
// void MyoControl::blinkLED(uint8_t ledPin, unsigned int repeat, unsigned int bTime) {
//     pinMode(ledPin, OUTPUT);
//     unsigned int i;
//     for(i=0;i<repeat;i++) {
//         digitalWrite(ledPin,HIGH);
//         delay(bTime);
//         digitalWrite(ledPin,LOW);
//         delay(bTime);
//     }
// }

/* sampling reads the ADC every 1 ms with the intervaltimer interrupt. */
void MyoControl::calibrationSampling() {
    emg = analogRead(emgpin);
    sampleOk = true; // sampleOk indicates that a new sample is ready to be processed
    noInterrupts();
}

/* Samples ADC every 1 ms, performs baseline rectification, Tunstin approximation,
and returns to main.ino
*/
double MyoControl::sampling() {
    emg = analogRead(emgpin);
  // double emgBaseline = emg*adcConv-emgMean;
    double emgBaseline = emg-emgMean;
    double emgRectify = abs(emgBaseline);
    double emgFilt = alpha1*emg_f_prev + alpha2*(emgRectify+emg_u_prev);
    // bufferArray[sampleCounter] = emgFilt;
    // Serial.print(emgpin);
    // Serial.print(": ");
    // Serial.print(emgFilt);
    // Serial.print(", ");
    // sampleCounter++;
    emg_f_prev = emgFilt;
    emg_u_prev = emgRectify;
    return emgFilt;
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
            // emgMean = emgMean + emg*adcConv;
            emgMean = emgMean + emg;

        }
        interrupts();
    }
    i = 0;
    emgMean = emgMean/meanSamples;
    Serial.print("EMG Mean is: ");
    Serial.println(emgMean);
}

/* mvcCalc computes the maximum voluntary contraction, the maximum force the
user is able to exert. This value is used to compute the activation threshold */
void MyoControl::mvcCalc(unsigned int mvcSamples) {
    unsigned int i = 0;
    while(i < mvcSamples) {
        delayMicroseconds(50);
        if(sampleOk) {
            sampleOk = false;
            i++;
            double emgBaseline = emg-emgMean;
            double emgRectify = abs(emgBaseline);
            double emgFilt = alpha1*emg_f_prev + alpha2*(emgRectify+emg_u_prev);
            emgMVC = emgMVC + emgFilt;
            emg_f_prev = emgFilt;
            emg_u_prev = emgRectify;
        }
        interrupts();
    }
    emgMVC = emgMVC/mvcSamples;
    emg_f_prev = 0;
    emg_u_prev = 0;
    Serial.print("EMG MVC is: ");
    Serial.println(emgMVC);
}

double MyoControl::slopeCalc(int muscle) {
    slope = (muscle*90-75)/(emgMVC-(muscle*3));
    return slope;
}

double MyoControl::interceptCalc(int muscle) {
    double intercept;
    intercept = muscle*90-slope*emgMVC;
    return intercept;
}

void MyoControl::calibration() {
    /* System calibration */
    /* Calibration step #1: calculate the baseline of the signal during 10 s */
    // blinkLED(13,1,500); // LED blinks once to indicate calibration step #1 start
    Serial.println("Calibration: keep muscles relaxed for 10 s");
    meanCalc(10000);
    Serial.println("Relaxed calibration complete");
    // blinkLED(13,1,500); // LED blinks once to indicate calibration step #1 end
    delay(1000);
    /* Calibration step #2: calculate the maximum voluntary contraction during 5 s*/
    // blinkLED(13,2,500); // LED blinks twice to indicate calibration step #2 start
    Serial.println("Calibration: perform MVC for 5 seconds");
    mvcCalc(5000);
    Serial.println("MVC calibration complete");
    // blinkLED(13,2,500); // LED bliks twice to indicate calibration step #2 end
    delay(1000);
}

/*
Prints the raw emg data followed by the filtered emg data in the format
rawemg1, filteremg1, rawemg2, filteremg2
// */
// void MyoControl::printSamples() {
//     // delayMicroseconds(50);
//     // Serial.print(emg*adcConv);
//     // Serial.print(", ");
//     if (sampleCounter == 200) {
//         noInterrupts();
//         delayMicroseconds(50);
//         for (uint8_t i = 0; i < 200; i++) {
//             Serial.print(bufferArray[i]);
//             Serial.print(", ");
//         }
//         Serial.println();
//         interrupts();
//     }
// }

// void MyoControl::activation() {
//     delayMicroseconds(50);
//     if(sampleOk) {
//         sampleOk = false;
//         // double emgMovav = movAv(); // Gets the amplitude of the measured EMG signal
//         /* If the amplitude of the EMG signal is greater than the activation threshold
//         (a 35% of the MVC), there is a muscle activation. */
//         // if(emgMovav > 0.35*emgMvc) {
//             // isActive = true;
//         // }
//         // /* If the amplitude of the EMG signal is below the activation threshold,
//         // there is no muscle activation. */
//         // else {
//         //     isActive = false;
//         // }
//     }
//     // return isActive;
// }

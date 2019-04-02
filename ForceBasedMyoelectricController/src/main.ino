/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * This code is distributed under a GPL 3.0 license
 */

#include <MyoControl.h>
#include <Arduino.h>
#include <IntervalTimer.h>

int channel1 = A0;
int channel2 = A1;

double processedDataArrCh1[200];
double processedDataArrCh2[200];
volatile unsigned int sampleCounter = 0;
unsigned int slidingWindow = 0;
double m_extensor;
double m_flexor;
double b_extensor;
double b_flexor;

MyoControl EMG_Channel1(channel1);
MyoControl EMG_Channel2(channel2);

/* Interval Timer allows interrupt to be called every 1 ms to set consistent
sampling frequency of 1000 Hz
*/
IntervalTimer calibrationTimer;
IntervalTimer initialTimer;

void calibrationSampling() {
    EMG_Channel1.calibrationSampling();
    EMG_Channel2.calibrationSampling();
}

void functionSampling() {
    double emg1 = EMG_Channel1.sampling();
    double emg2 = EMG_Channel2.sampling();
    delayMicroseconds(50);
    Serial.print("DATA, TIME,");
    Serial.print(emg1);
    Serial.print(" , ");
    Serial.println(emg2);
    processedDataArrCh1[sampleCounter+slidingWindow] = emg1;
    processedDataArrCh2[sampleCounter+slidingWindow] = emg2;
    sampleCounter++;
}

int classifier(double emgDifferential) {
    int y;

    if (emgDifferential < -3) {
        y = m_flexor*emgDifferential-b_flexor;
    } else if (emgDifferential > 3) {
        y = m_extensor*emgDifferential-b_extensor;
    } else {
        y = 30*emgDifferential;
    }

    return (int)y;
}

int contractionPulseMap(int contraction) {
    int pulseWidth;
    if (contraction > 0) { //extensor
        pulseWidth = -3.75*contraction + 1500;
    } else if (contraction < 0) { //flexor
        pulseWidth = -7.50*contraction + 1500;
    } else {
        pulseWidth = 1500;
    }
    return pulseWidth;
}

void setup() {
    delay(3000); //delay 2 seconds to open up window
    Serial.println("Successful Upload: Starting Program");
    Serial.begin(14400);
    Serial.println("LABEL,Time, MuscleA1");
    Serial.println("RESETTIMER"); //resets timer to 0
    //Calibration

    calibrationTimer.begin(calibrationSampling,1000); //samples every 1000 microseconds
    delay(5000);
    Serial.println("Calibrating channel 1:");
    delay(1000);
    EMG_Channel1.calibration();
    Serial.println("Channel 1 calibrated");
    Serial.println("Calibrating channel 2:");
    delay(1000);
    EMG_Channel2.calibration();
    Serial.println("Channel 2 calibrated");
    calibrationTimer.end(); //Stops sampling of calibration period
    m_extensor = EMG_Channel1.slopeCalc(1); //1 indicates positive 90 used as mvcCalc
    m_flexor = EMG_Channel2.slopeCalc(-1);
    b_extensor = EMG_Channel1.interceptCalc(1);
    b_flexor = EMG_Channel2.interceptCalc(-1);
    //Function
    Serial.println("Calibration complete: begin function in 5 seconds");
    delay(5000);
    initialTimer.begin(functionSampling,1000);
}

void loop() {
    if (sampleCounter == 49) {
        noInterrupts();
        double ch1sum = 0;
        double ch2sum = 0;
        for (unsigned int i = 0; i < 199; i++) {
          Serial.print("DATA, TIME,");
          Serial.println(processedDataArrCh2[i]);
            ch1sum += processedDataArrCh1[i];
            ch2sum += processedDataArrCh2[i];
        }
        double ch1MAV = ch1sum/200;
        double ch2MAV = ch2sum/200;
        double emgDifferential = ch1MAV - ch2MAV;
        int contraction = classifier(emgDifferential);
        int pulseWidth = contractionPulseMap(contraction);
        sampleCounter = 0;
        if (slidingWindow == 150) {
            slidingWindow = 0;
        } else {
            slidingWindow += 50;
        }
        interrupts();
    }
}

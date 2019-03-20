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

double processedDataArr[2][200];
unsigned int sampleCounter = 0;
unsigned int slidingWindow = 0;

MyoControl EMG_Channel1(channel1);
MyoControl EMG_Channel2(channel2);

/* Interval Timer allows interrupt to be called every 1 ms to set consistent
sampling frequency of 1000 Hz
*/
IntervalTimer calibrationTimer;
IntervalTimer functionTimer;

void calibrationSampling() {
    EMG_Channel1.calibrationSampling();
    EMG_Channel2.calibrationSampling();
}

void sample() {
    double emg1 = EMG_Channel1.sampling();
    double emg2 = EMG_Channel2.sampling();
    processedDataArr[1][sampleCounter+slidingWindow];
    processedDataArr[2][sampleCounter+slidingWindow];
    sampleCounter++;
}

void setup() {
    Serial.println("Successful Upload: Starting Program");
    Serial.begin(115200);
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
    //Function
    Serial.println("Calibration complete: begin function in 5 seconds");
    delay(5000);
    functionTimer.begin(sample,1000);
    while(sampleCounter < 199) {
    }
    sampleCounter == 49;
}

void loop() {
    if (sampleCounter == 49) {
        noInterrupts();
        double ch1sum = 0;
        double ch2sum = 0;
        for (unsigned int i = 0; i < 199; i++) {
            ch1sum += bufferArray[1][i];
            ch2sum += bufferArray[2][i];
        }
        double ch1MAV = ch1sum/200;
        double ch2MAV = ch2sum/200;
    }
}

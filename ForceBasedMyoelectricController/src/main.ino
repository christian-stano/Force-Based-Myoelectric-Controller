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

void sampling() {
    EMG_Channel1.sampling();
    EMG_Channel2.sampling();
}

void setup() {
    Serial.begin(115200);
    //Calibration
    calibrationTimer.begin(sampling,1000); //samples every 1000 microseconds
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
    functionTimer.begin(sampling,1000);
    delay(5000);
}

void loop() {
    Serial.println("EMG Channel 1 Values: ");
    EMG_Channel1.printSamples();
    Serial.println("EMG Channel 1 Values: ");
    EMG_Channel2.printSamples();
}

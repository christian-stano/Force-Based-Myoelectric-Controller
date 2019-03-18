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

IntervalTimer myTimer;

void sampling() {
    EMG_Channel1.sampling();
    EMG_Channel2.sampling();
}

void setup() {
    Serial.begin(115200);
    myTimer.begin(sampling,1000); //samples every 1000 microseconds
    delay(5000);
    Serial.println("Calibrating channel 1:");
    delay(1000);
    EMG_Channel1.calibration();
    Serial.println("Channel 1 calibrated");
    Serial.println("Calibrating channel 2:");
    delay(1000);
    EMG_Channel2.calibration();
    Serial.println("Channel 2 calibrated");
    Serial.println("Calibration complete: begin function in 5 seconds");
    delay(5000);
    // EMG_Channel1.printSamples();
    EMG_Channel2.printSamples();
}

void loop() {
//     for (uint8_t i = 0; i < 3000; i++) {
//     EMG_Channel1.printSamples();
//     Serial.print(", ");
//     EMG_Channel2.printSamples();
// }   Serial.println();

}

#include <MyoControl.h>

/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * This code is distributed under a GPL 3.0 license
 */

#include <Arduino.h>
#include <MsTimer2.h>

int channel1 = A0;
int channel2 = A1;

MyoControl EMG_Channel1(channel1);
MyoControl EMG_Channel2(channel2);

void sampling() {
    EMG_Channel1.sampling();
    EMG_Channel2.sampling();
}

void setup() {
  Serial.begin(115200);
  MsTimer2::set(1,sampling);
  MsTimer2::start();
  delay(5000);
  Serial.println("Calibrating channel 1");
  delay(1000);
  EMG_Channel1.calibration();
  Serial.println("Calibrating channel 2");
  delay(1000);
  EMG_Channel2.calibration();
}

void loop() {
  // EMG_Channel1.activation();
  // EMG_Channel2.activation();
}

/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * This code is distributed under a GPL 3.0 license
 */

#include <MyoControl.h>
#include <Arduino.h>
#include <IntervalTimer.h>
#include <Servo.h>
// #include <PID_v1.h>

// Initialize relevant servo elements
int servoPin = A2;//Teensy pin communicating with Servo
Servo Servo1; // create servo object for arduino library implementation

// Initialize Relevant PID elements
double setpoint, pulseWidthPID, pulseWidth, pulseWidthPID2;
int prev_contraction = 0;
double prev_pulsewidth = 0;
double prev_error = 0;
double prev_error2 = 0;

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
int contractionPrev = 0;

MyoControl EMG_Channel1(channel1);
MyoControl EMG_Channel2(channel2);

/* Interval Timer allows interrupt to be called every 1 ms to set consistent
sampling frequency of 1000 Hz
*/
IntervalTimer calibrationTimer;
IntervalTimer initialTimer;
IntervalTimer MVCTimer;

void calibrationSampling() {
    EMG_Channel1.calibrationSampling();
    EMG_Channel2.calibrationSampling();
}

void functionSampling() {
    double emg1 = EMG_Channel1.sampling();
    double emg2 = EMG_Channel2.sampling();
    delayMicroseconds(50);
    processedDataArrCh1[sampleCounter+slidingWindow] = emg1;
    processedDataArrCh2[sampleCounter+slidingWindow] = emg2;
    sampleCounter++;
}

int classifier(double emgDifferential) {
    int y;

    if (emgDifferential < -3) {
        y = m_flexor*emgDifferential+b_flexor;
    } else if (emgDifferential > 3) {
        y = m_extensor*emgDifferential+b_extensor;
    } else {
        y = 30*emgDifferential;
    }

    return (int)y;
}

double contractionPulseMap(int contraction) {
    // double pulseWidth;
    if (contraction > 0) { //extensor
        pulseWidth = -3.75*contraction + 1500;
    } else if (contraction < 0) { //flexor
        pulseWidth = -7.50*contraction + 1500;
    } else {
        pulseWidth = 1500;
    }
    return pulseWidth;
}

double MVCCalibration(int mvcSamples) {
    int i = 0;
    double emgMVC = 0;
    while (i < mvcSamples) {
        if (sampleCounter == 49) {
            noInterrupts();
            double ch1sum = 0;
            double ch2sum = 0;
            for (unsigned int i = 0; i < 199; i++) {
                ch1sum += processedDataArrCh1[i];
                ch2sum += processedDataArrCh2[i];
            }
            double ch1MAV = ch1sum/200;
            double ch2MAV = ch2sum/200;
            double emgDifferential = ch1MAV - ch2MAV;
            emgMVC = emgMVC + emgDifferential;
            sampleCounter = 0;
            if (slidingWindow == 150) {
                slidingWindow = 0;
            } else {
                slidingWindow += 50;
            }
            i++;
            interrupts();
        }
    }
    emgMVC = emgMVC/mvcSamples;
    Serial.print("EMG MVC is: ");
    Serial.println(emgMVC);
    return emgMVC;
}

void setup() {
    Servo1.attach(servoPin); // attach servo to pin prior to use in code
    setpoint = 1500;

    delay(5000); //delay 8 seconds to open up window
    Serial.println("Successful Upload: Starting Program");
    Serial.begin(14400);

    Serial.println("LABEL,Time, MuscleA1, label2, label3");
    Serial.println("RESETTIMER");
    // Calibration

    calibrationTimer.begin(calibrationSampling,1000); //samples every 1000 microseconds
    delay(3000);
    Serial.println("Calibrating channel 1:");
    delay(1000);
    EMG_Channel1.calibration();
    Serial.println("Channel 1 calibrated");
    Serial.println("Calibrating channel 2:");
    delay(1000);
    EMG_Channel2.calibration();
    Serial.println("Channel 2 calibrated");
    calibrationTimer.end(); //Stops sampling of calibration period

    //Classifier Calibration via mvcCalc
    Serial.println("Begin MVC Calibration for Classifier");
    Serial.println("Perform MVC of extensor for 5 seconds");
    initialTimer.begin(functionSampling,1000);
    double extensorMVC = MVCCalibration(140);
    noInterrupts();
    Serial.println("Extensor MVC calibration complete");
    Serial.println("Perform MVC of flexor for 5 seconds");
    interrupts();
    double flexorMVC = MVCCalibration(140);
    noInterrupts();
    Serial.println("Flexor MVC calibration complete");

    m_extensor = EMG_Channel1.slopeCalc(1,extensorMVC); //1 indicates positive 90 used as mvcCalc
    m_flexor = EMG_Channel2.slopeCalc(-1,flexorMVC);
    b_extensor = EMG_Channel1.interceptCalc(1);
    b_flexor = EMG_Channel2.interceptCalc(-1);
    //Function
    Serial.println("Calibration complete: begin function in 5 seconds");
    interrupts();
    // initialTimer.begin(functionSampling,1000);
}

void loop() {
    if (sampleCounter == 49) {
        noInterrupts();
        double ch1sum = 0;
        double ch2sum = 0;
        for (unsigned int i = 0; i < 199; i++) {
            ch1sum += processedDataArrCh1[i];
            ch2sum += processedDataArrCh2[i];
        }
        double ch1MAV = ch1sum/200;
        double ch2MAV = ch2sum/200;
        double emgDifferential = ch1MAV - ch2MAV;
        int contraction = classifier(emgDifferential);
        pulseWidth = contractionPulseMap(contraction);
        int threshold = abs(contraction - contractionPrev);

        //Implement PID Considerations

              //if (contract < 2 && contract > -2){
                //change_contract = -prev_contraction ;
                // pulseWidth = contractionPulseMap (change_contract);
            //} else {
              double pk = 0;
              double ik = 0;
              double dk = 4;

              double PID_in2 = pulseWidth; // without feedback

              double error_present2 = - setpoint + PID_in2;
              double prop_factor2 = pk * error_present2;

              double error_past2 = error_present2 + prev_error2;
              double integral_factor2 = ik * error_past2;

              double error_future2 = error_present2 - prev_error2;
              double diff_factor2 = dk *error_future2;

              pulseWidthPID2 = setpoint + diff_factor2 + integral_factor2 + prop_factor2;

            //  Servo1.writeMicroseconds(pulseWidthPID);
             //}
           //prev_contraction = contraction;
           prev_error2 = error_present2;

        if (pulseWidth < 2250 && pulseWidth > 750) {
            if (contraction > 75 || contraction < -75) {
                if (threshold > 10) {
                    Servo1.writeMicroseconds(pulseWidth);
                }
            } else {
                if (threshold > 5) {
                    Servo1.writeMicroseconds(pulseWidth);
                }
            }
        }

        // Serial.print("DATA, TIME, ");
        // Serial.print(emgDifferential);
        // Serial.print(" , ");
        // Serial.print(contraction);
        // Serial.print(" , ");
        // Serial.print(pulseWidth);
        // Serial.print(" , ");
        // Serial.print(pulseWidthPID2);
        // Serial.print(" , ");
        // Serial.println(pulseWidthPID);

        sampleCounter = 0;
        if (slidingWindow == 150) {
            slidingWindow = 0;
        } else {
            slidingWindow += 50;
        }
        interrupts();
    }
}

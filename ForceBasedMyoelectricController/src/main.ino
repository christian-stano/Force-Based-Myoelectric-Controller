/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * This code is distributed under a GPL 3.0 license
 */

#include <MyoControl.h>
#include <Arduino.h>
#include <IntervalTimer.h>
#include <Servo.h> // h file for servo communication

// Initialize relevant servo elements
int servoPin = A2;//Teensy pin communicating with Servo
Servo Servo1; // create servo object for arduino library implementation

// Initialize Relevant PID elements
double setpoint, pulseWidth, pulseWidthPID2;
double prev_pulsewidth = 0; // initial pulseWidth comparator
double prev_error2 = 0; // intial error comparator

int channel1 = A0; // Extensor Channel
int channel2 = A1; // Flexor Channel

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

    if (emgDifferential < -2.5) {
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

    setpoint = 1500; // The zero setpoint, or pulseWidth that results in relaxed position

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
        contractionPrev = contraction;


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

        //Implementation of CLINICAL PID Considerations

          double pk = 0; // Proportional Constant for PID
          double ik = 0; // Integral Constant for PID
          double dk = 4; // Differential Constant for PID

            // Error Term Calculations for Decline to Position Maintenance
            double PID_in2 = pulseWidth; // input to PID for manipualtion from the above software, is the pulseWidth (no feedback, given control scheme)

            double error_present2 = - setpoint + PID_in2; // approximation of proportional error
            double prop_factor2 = pk * error_present2; // approximation of proportional PID Factor

            double error_past2 = error_present2 + prev_error2; // approximation of integral error
            double integral_factor2 = ik * error_past2; // approximation of integral PID Factor

            double error_future2 = error_present2 - prev_error2; // approximation of derivative error
            double diff_factor2 = dk *error_future2; // approximation of differential PID Factor

            // Redefine puslewidth given the error summation from setpoint
            pulseWidthPID2 = setpoint + diff_factor2 + integral_factor2 + prop_factor2;

            prev_error2 = error_present2; // redefin the previous error for next loop consideration

            // If the system is changing position, or threshold has been met, as defined by the percieved contraction
            // The PID will reset with previous error = 0 and the outputted pulsewidth = to the required change in position.
            if (contraction > 75 || contraction < -75) {
                // If in maximum regions, change in position only initiated if threshold > 10
                if (threshold > 10) {
                    // Total Change in position will be the difference between the two contraction states
                    double change_contract = contraction-contractionPrev;
                    pulseWidthPID2 = contractionPulseMap(change_contract); // map change in contraction to pulseWidth
                    prev_error2 = 0; // reset PID for future maintenance of position
                }
            }
            else {
                // If in constant, linear classifier region, change in position only initiated if threshold > 5
                if (threshold > 5) {
                   // Total Change in position will be the difference between the two contraction states
                    double change_contract = contraction-contractionPrev ;
                    pulseWidthPID2 = contractionPulseMap(change_contract);// map change in contraction to pulseWidth
                    prev_error2 = 0; // reset PID for future maintenance of position
                }
            }

            double pW_threshold = abs(prev_pulsewidth - pulseWidthPID2);// Use pulsewidth difference to threshold output changes, and remove flutter from baseline noise
            prev_pulsewidth = pulseWidthPID2; // set the previous pulsewidth for the next loop

            // Check PID output to ensure that it is not outside of the pulseWidth communication that can be handled by the prosthetic
            if (pulseWidthPID2 < 2250 && pulseWidthPID2 > 750) {
              // Threshold is changed based on the level of contraction, to ensure artifacts are not lost when thresholding pulse widths
                if (contraction > 75 || contraction < -75) {
                    if (pW_threshold > 10) {
                        Servo1.writeMicroseconds(pulseWidthPID2); // if pulsewidth outside of threshold change write to object
                    }
                } else {
                    if (pW_threshold > 5) {
                        Servo1.writeMicroseconds(pulseWidthPID2); // if pulsewidth outside of threshold change write to object
                    }
                }
            }

      // Implementation of PID for SERVO CONSTRUCT
            // PID constant definition
            //   double pk = 0; //proportional
            //   double ik = 0; // integral
            //   double dk = 4; //differential
            //
            //   double PID_in2 = pulseWidth; // input to PID = to pulseWidth output from above
            //
            //   double error_present2 = - setpoint + PID_in2; // current error is the difference between setpoint an PID input
            //   double prop_factor2 = pk * error_present2; // approximation of porportional PID factor
            //
            //   double error_past2 = error_present2 + prev_error2; // approximation of integral error
            //   double integral_factor2 = ik * error_past2; // approximation of integral PID factor
            //
            //   double error_future2 = error_present2 - prev_error2; // approximation of differential error
            //   double diff_factor2 = dk *error_future2; // approximation of differential PID factor
            //
            //   pulseWidthPID2 = setpoint + diff_factor2 + integral_factor2 + prop_factor2;/sum error with setpoint for pulseWidth output
            //   prev_error2 = error_present2; //reset previous error



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

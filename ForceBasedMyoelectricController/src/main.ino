/*
 * Copyright 2019 Autofabricantes
 * Author: Alvaro Villoslada (Alvipe)
 * This file is part of Myoware-servo-control (https://github.com/Autofabricantes/Myoware-servo-control).
 * This code is distributed under a GPL 3.0 license
 */

 /*
 * Force Based Controller for Myoelectric Prostheses
 * Authors: Christian Stano, Allyson King
 * Email: cstano29@gmail.com
 * This code and the associated libraries were developed by students at Vanderbilt University
 under the guidance of Emily Gracyzk, Ph.D of Case Western Reserve University
 * This software package translates raw EMG signals collected by two MyoWare
 EMG sensors to Servo or prosthetic hand actuation. It includes sampling at 1000 Hz,
 digital signal pre-processing via rectifying and a 1 Hz low pass smoothing filter,
 a 200 ms sliding window that calculates the differential MAV, a contraction intent
 classifier, and subsequent mapping to PWM for a Servo or prosthetic hand
 */

#include <MyoControl.h> // h file for custom developed MyoWare
#include <Arduino.h> // h file for Arduino environment
#include <IntervalTimer.h> // h file interrupt timer for 1000 Hz sampling
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

double processedDataArrCh1[200]; // 200 ms window for extensor channel (contains pre-processed doubles)
double processedDataArrCh2[200]; // 200 ms window for flexor channel (contains pre-processed doubles)
// sampleCounter to keep track of when processing should occur (every 50 ms)
// volatile type used because changed during interrupt
volatile unsigned int sampleCounter = 0;
unsigned int slidingWindow = 0; // position in processedDataArr
double m_extensor; // slope for piecewise classifier above 75% extensor contraction
double m_flexor; // slope for piecewise classifier below -75% flexor contraction
double b_extensor; // intercept for piecewise classifier above 75% extensor contraction
double b_flexor;  // intercept for piecewise classifier below -75% flexor contraction
int contractionPrev = 0; // previous window's contraction level for use in thresholding

MyoControl EMG_Channel1(channel1); // MyoControl object for extensor
MyoControl EMG_Channel2(channel2); // MyoControl object for flexor

/* Interval Timer allows interrupt to be called every 1 ms to set consistent
sampling frequency of 1000 Hz
*/
IntervalTimer calibrationTimer;
IntervalTimer initialTimer;

/*
 * Calibration interrupt function
 * Calls the MyoControl calibrationsampling() function every ms for both channels
 * Used to find the mean of the signal during rest for baseline removal
*/
void calibrationSampling() {
    EMG_Channel1.calibrationSampling();
    EMG_Channel2.calibrationSampling();
}

/*
 * Function interrupt
 * Calls sampling sampling() function from MyoControl every ms for each channel
 * Used to maintain 1000 Hz sampling rate during MVC calibration and regular function
 * Inputs resulting pre-processed EMG data point into 200 ms array at point sampleCounter+slidingWindow
 * Output: fills 200 ms array until sampleCounter = 49
*/
void functionSampling() {
    double emg1 = EMG_Channel1.sampling();
    double emg2 = EMG_Channel2.sampling();
    delayMicroseconds(50);
    processedDataArrCh1[sampleCounter+slidingWindow] = emg1;
    processedDataArrCh2[sampleCounter+slidingWindow] = emg2;
    sampleCounter++;
}

/*
 * Piecewise linear classifier function
 * Input: EMG MAV differential between extensor and flexor channels
 * Note: uses dynamically adapted line from MVC calibration if differential MAV
 * > 2.5 (> 75% contraction) or < -2.5 (< 75% contraction)
 * Output: predicted contraction level (int)
*/
int classifier(double emgDifferential) {
    int y;

    if (emgDifferential < -2.5) {
        y = m_flexor*emgDifferential+b_flexor;
    } else if (emgDifferential > 2.5) {
        y = m_extensor*emgDifferential+b_extensor;
    } else {
        y = 30*emgDifferential;
    }

    return (int)y;
}

// Maps contraction level to PWM for servo
double contractionPulseMap(int contraction) {
    if (contraction > 0) { //extensor
        pulseWidth = -3.75*contraction + 1500;
    } else if (contraction < 0) { //flexor
        pulseWidth = -7.50*contraction + 1500;
    } else {
        pulseWidth = 1500;
    }
    return pulseWidth;
}

/*
 * MVC Calibration function
 * This function calculates the MVC over a period of contraction by the user
 * which is used to dynamically adjust the classifier function for contractions
 * > 75% or < 75%
 * Input: mvcSamples- number of iterations of MVC calculations to perform
 * Output: average MVC over mvcSamples iterations of calculation
*/
double MVCCalibration(int mvcSamples) {
    int i = 0;
    double emgMVC = 0;
    while (i < mvcSamples) {
        if (sampleCounter == 49) { //process array every 50 ms
            noInterrupts(); // pause interrupts until calculation complete
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
            interrupts(); // resume interrupts once loop completed
        }
    }
    emgMVC = emgMVC/mvcSamples; //average MVC
    Serial.print("EMG MVC is: ");
    Serial.println(emgMVC);
    return emgMVC;
}

void setup() {
    Servo1.attach(servoPin); // attach servo to pin prior to use in code

    setpoint = 1500; // The zero setpoint, or pulseWidth that results in relaxed position

    delay(5000); //delay 5 seconds to open up window
    Serial.println("Successful Upload: Starting Program");
    Serial.begin(14400); // Serial baud rate at 14400

    // Calibration
    calibrationTimer.begin(calibrationSampling,1000); //samples every 1000 microseconds or at 1000 Hz
    delay(3000); //delay for stability
    Serial.println("Calibrating channel 1:"); //extensor calibration
    delay(1000);
    EMG_Channel1.calibration();
    Serial.println("Channel 1 calibrated");
    Serial.println("Calibrating channel 2:");
    delay(1000);
    EMG_Channel2.calibration();
    Serial.println("Channel 2 calibrated");
    calibrationTimer.end(); //Stops sampling of calibration period

    //Classifier Calibration via MVCCalibration
    Serial.println("Begin MVC Calibration for Classifier");
    Serial.println("Perform MVC of extensor for 5 seconds");
    initialTimer.begin(functionSampling,1000); // begin second interrupt timer
    double extensorMVC = MVCCalibration(140); // MVC for approximately 7 seconds
    noInterrupts();
    Serial.println("Extensor MVC calibration complete");
    Serial.println("Perform MVC of flexor for 5 seconds");
    interrupts();
    double flexorMVC = MVCCalibration(140);
    noInterrupts();
    Serial.println("Flexor MVC calibration complete");

    // Upper and Lower Bound calculation of linear piecewise function
    m_extensor = EMG_Channel1.slopeCalc(1,extensorMVC); //1 indicates positive 90% contraction of extensor
    m_flexor = EMG_Channel2.slopeCalc(-1,flexorMVC); //-1 indicates negative 90% contraction of flexor
    b_extensor = EMG_Channel1.interceptCalc(1);
    b_flexor = EMG_Channel2.interceptCalc(-1);
    //Function
    Serial.println("Calibration complete: begin function in 5 seconds");
    interrupts();
}

/*
 * loop() function
 * Continuously runs throughout function of program, recalculating MAV differential,
 contraction level, and output every 50 ms over the 200 ms processedDataArr
*/
void loop() {
    if (sampleCounter == 49) {
        noInterrupts(); //pause interrupts during calculations for stability
        double ch1sum = 0;
        double ch2sum = 0;
        for (unsigned int i = 0; i < 199; i++) {
            ch1sum += processedDataArrCh1[i];
            ch2sum += processedDataArrCh2[i];
        }
        double ch1MAV = ch1sum/200;
        double ch2MAV = ch2sum/200;
        double emgDifferential = ch1MAV - ch2MAV;
        int contraction = classifier(emgDifferential); //gesture classifier
        pulseWidth = contractionPulseMap(contraction); //map contraction to pulsewidth
        int threshold = abs(contraction - contractionPrev); //calculate threshold
        contractionPrev = contraction;

        // Servo actuation
        if (pulseWidth < 2250 && pulseWidth > 750) { //check if pulseWidth within saturation bounds
            if (contraction > 75 || contraction < -75) { // higher threshold within bounds to prevent flutter
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

            // Check PID output to ensure that it is not outside of the pulseWidth communication that can be handled by the prosthetic
            // if (pulseWidthPID2 < 2250 && pulseWidthPID2 > 750) {
            //   // Threshold is changed based on the level of contraction, to ensure artifacts are not lost when thresholding pulse widths
            //     if (contraction > 75 || contraction < -75) {
            //         if (pW_threshold > 10) {
            //             Servo1.writeMicroseconds(pulseWidthPID2); // if pulsewidth outside of threshold change write to object
            //         }
            //     } else {
            //         if (pW_threshold > 5) {
            //             Servo1.writeMicroseconds(pulseWidthPID2); // if pulsewidth outside of threshold change write to object
            //         }
            //     }
            // }

        sampleCounter = 0; //reset sampleCounter
        if (slidingWindow == 150) {
            slidingWindow = 0;
        } else {
            slidingWindow += 50;
        }
        interrupts(); //resume sampling once calculation complete 
    }
}

// #include "Arduino.h"
// #include "MyoControl.h"
// #include "Processing.h"
//
// sampleCounter = 0; //determines index of array to populate with data point
// b1 = true; // b1-b4 indicates what section of buffer array is new and old data to be populated or flush
// b2 = false;
// b3 = false;
// b4 = false;
//
// //Manages insertion of each EMG data point into correct position of 200 element buffer
// void MyoWare::bufferManager() {
//     if (b1) { //insert in 0-49
//         emgSum -= bufferArray[sampleCounter]; //Remove oldest data point from running average
//         bufferArray[sampleCounter] = emg * adcConv;
//         emgSum += bufferArray[sampleCounter]; //Add newest data point to running average
//         sampleCounter++;
//     } else if (b2) { //insert in 50-99
//         emgSum -= bufferArray[sampleCounter+50];
//         bufferArray[sampleCounter+50] = emg * adcConv;
//         emgSum += bufferArray[sampleCounter+50];
//         sampleCounter++;
//     } else if (b3) { //insert in 100-149
//         emgSum -= bufferArray[sampleCounter+100];
//         bufferArray[sampleCounter+100] = emg * adcConv;
//         emgSum += bufferArray[sampleCounter+100];
//         sampleCounter++;
//     } else { //insert in 150-199
//         emgSum -= bufferArray[sampleCounter+150];
//         bufferArray[sampleCounter+150] = emg * adcConv;
//         emgSum += bufferArray[sampleCounter+150];
//         sampleCounter++;
//     }
//
//     //cycles 50 ms populating partition of buffer array every 50 ms
//     if (sampleCounter == 49) {
//         sampleCounter = 0;
//         meanCalc();
//         if (b1) {
//             b1 = false;
//             b2 = true;
//         } else if (b2) {
//             b2 = false;
//             b3 = true;
//         } else if (b3) {
//             b3 = false;
//             b4 = true;
//         } else {
//             b4 = false;
//             b1 = true;
//         }
//     }
// }
//
// // //Calculate mean of 200 ms EMG signal to rectify with baseline of 0V
// // void MyoWare::meanCalc() {
// //     emgMean = emgSum/200;
// // }
// //
// // void MyoWare::MAV() {
// //     double rectSum = 0; //rectified sum of values
// //     for (unsigned int i = 0; i < 199; i++) {
// //         rectSum += abs(bufferArray[i] - emgMean);
// //     }
// //     Serial.println(rectSum/200); //returns MAV of 200 ms window
// // }
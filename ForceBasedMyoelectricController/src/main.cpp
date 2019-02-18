#include <Arduino.h>
#include <MsTimer2.h>
#include <MyoWare.h>

MyoWare EMG_Channel1(A0);
MyoWare EMG_Channel2(A1);

void sampling() {
    EMG_Channel1.sampling();
    EMG_Channel2.sampling();
}

void setup() {
  Serial.begin(115200);
  MsTimer2::set(1,sampling);
  MsTimer2::start();
}

void loop() {
  EMG_Channel1.sampling();
  EMG_Channel2.sampling();
}

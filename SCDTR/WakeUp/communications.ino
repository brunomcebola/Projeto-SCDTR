#include <Wire.h>

#include "constants.h"

int id = 1;
int x = 0;

void setup() {
    Serial.begin(115200);

    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire1.setSDA(10);
    Wire1.setSCL(11);

    Wire.setClock(100000);
    Wire1.setClock(100000);

    Wire.begin();

    Wire1.begin(id);
    Wire1.onRequest(requestEvent);  // register event
    Wire1.onReceive(receiveEvent);  // register event
}

void loop() {
    Wire.beginTransmission(4);  // transmit to device #4
    Wire.write("x is ");        // sends five bytes
    Wire.write(x);              // sends one byte
    Wire.endTransmission();     // stop transmitting
    x++;

    delay(500);
}
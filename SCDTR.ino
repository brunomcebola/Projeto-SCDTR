#include "constants.h"
#include "simulator.h"
#include "mixin.h"

Simulator simulator; 

void setup() {
    float G;

    Serial.begin(115200);
    analogReadResolution(12);        // default is 10
    analogWriteFreq(ANALOG_FREQ);  // 60KHz, about max
    analogWriteRange(ANALOG_MAX);  // 100% duty cycle

    delay(2500);

    G = calibrate_gain();

    simulator.set_simulator(M, B, G);
}

// loop function

void loop() {
    delay(2000);
    float v;

    int n_i = analogRead(A0);
    int n_f = 4096;

    simulator.set_simualtion(micros(), n_i, n_f);

    while(true) {
        v = simulator.simulate(micros());
        
        Serial.println(volt_to_lux(v, M, B));
    }
}

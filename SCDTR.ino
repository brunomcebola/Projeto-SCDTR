#include "constants.h"
#include "controller.h"
#include "mixin.h"
#include "pi.h"
#include "simulator.h"

const int sampling_time = 10;  // ms

Simulator simulator;
Controller controller;

long int initial_time = 0;
long int final_time = 0;

float u_sim = 0;
float u_real = 0;
float u_cont = 0;

void setup() {
    float G;

    Serial.begin(115200);
    analogReadResolution(12);      // default is 10
    analogWriteFreq(ANALOG_FREQ);  // 60KHz, about max
    analogWriteRange(ANALOG_MAX);  // 100% duty cycle

    delay(5000);

    G = calibrate_gain();

    controller.set_controller(sampling_time * pow(10, -3), 1.75, 5);

    simulator.set_simulator(M, B, G);

    controller.set_lux_ref(0);

    simulator.set_simualtion(
        micros(), analogRead(A0),
        lux_to_n(controller.get_lux_ref(), simulator.get_gain()));
}

// loop function

void loop() {
    initial_time = micros();
    
    interface();

    u_sim = simulator.simulate(micros());
    u_real = n_to_volt(analogRead(A0));

    controller.anti_wind_up();

    controller.calc_u_fb(u_sim, u_real);
    controller.calc_u_ff(simulator.get_gain());

    u_cont = controller.get_control_signal();

    analogWrite(LED_PIN, volt_to_n(u_cont));

    final_time = micros();

    delay(sampling_time -
          ((final_time - initial_time) * pow(10, -3)));  // sampling time;
}

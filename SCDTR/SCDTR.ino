#include "circular_buffer.h"
#include "constants.h"
#include "controller.h"
#include "mixin.h"
#include "simulator.h"

const int sampling_time = 10;  // ms

Simulator simulator;
Controller controller;
CircularBuffer<6000> buffer;
buffer_data data;

long int initial_time = 0;
long int final_time = 0;

float u_sim = 0;
float u_real = 0;
float u_cont = 0;

char lum = '\0';
char cmd = '\0';
char sub_cmd = '\0';
int val_i = 0;
float val_f = 0.0f;
float duty_cycle = 0.0f;
float lux = 0;
bool stream_duty_cycle = false;
bool stream_lux = false;
bool buffer_duty_cycle = false;
bool buffer_lux = false;
int buffer_read_size = 0;
int buffer_read_counter = 0;

int iteration_counter = 0;
float flicker = 0.0f;

float total_energy = 0.0f;
float visibility_error = 0.0f;
float flicker_error = 0.0f;
float prev_lux_1 = 0.0f;
float prev_lux_2 = 0.0f;

void setup() {
    float G;

    Serial.begin(115200);
    analogReadResolution(12);      // default is 10
    analogWriteFreq(ANALOG_FREQ);  // 60KHz, about max
    analogWriteRange(ANALOG_MAX);  // 100% duty cycle

    delay(5000);

    G = calibrate_gain();

    delay(1000);

    controller.set_controller(sampling_time * pow(10, -3), 1.75, 5);

    simulator.set_simulator(M, B, G);

    controller.set_lux_ref(25);

    simulator.set_simualtion(
        micros(), analogRead(A0),
        lux_to_n(controller.get_lux_ref(), simulator.get_gain()));
}

// loop function

void loop() {
    initial_time = micros();

    iteration_counter++;

    // interface to receive comands from the computer
    interface();

    // simulate the sytem respose
    u_sim = simulator.simulate(micros());

    u_real = n_to_volt(analogRead(A0));

    // apply anti-windup to the system
    controller.anti_wind_up();

    // compute the feedback control signal
    controller.calc_u_fb(u_sim, u_real);

    // compute the feedforward control signal
    controller.calc_u_ff(simulator.get_gain());

    // compute the new control signal
    u_cont = controller.get_control_signal();

    // change LED intensity
    analogWrite(LED_PIN, volt_to_n(u_cont));

    // store duty cycle and lux in buffer
    duty_cycle = u_cont / V_REF;
    lux = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);
    data = {duty_cycle, lux};
    buffer.insert_new(data);

    // compute the visibility error (not averaged)
    visibility_error += max(0, controller.get_lux_ref() - lux);

    // compute accumulated energy comsuption before changing the control signal
    total_energy += NOMINAL_POWER * duty_cycle * sampling_time * pow(10, -3);

    // compute the total flicker error
    if (iteration_counter > 2) {
        if ((lux - prev_lux_1) * (prev_lux_1 * prev_lux_2) < 0) {
            flicker = (abs(lux - prev_lux_1) + abs(prev_lux_1 * prev_lux_2)) /
                      (2 * sampling_time * pow(10, -3));
        } else {
            flicker = 0.0f;
        }

        flicker_error += flicker;

        prev_lux_2 = prev_lux_1;
        prev_lux_1 = lux;
    } else if (iteration_counter == 2) {
        prev_lux_1 = lux;
    } else {
        prev_lux_2 = lux;
    }

    // stream duty cycle
    if (stream_duty_cycle) {
        Serial.printf("s d %c %f %d\n", lum, duty_cycle, millis());
    }

    // stream lux
    if (stream_lux) {
        Serial.printf("s l %c %f %d\n", lum, lux, millis());
    }

    // read buffer
    if (buffer_read_counter < buffer_read_size) {
        int t = 20;
        if (buffer_read_size - buffer_read_counter < t) {
            t = buffer_read_size - buffer_read_counter;
        }

        for (int i = 0; i < t; i++) {
            data = buffer.remove_oldest();

            if (buffer_duty_cycle) {
                Serial.printf("%f, ", data.duty_cycle);
            }

            if (buffer_lux) {
                Serial.printf("%f, ", data.lux);
            }

            buffer_read_counter++;
        }
    } else {
        if (buffer_duty_cycle || buffer_lux) {
            Serial.println();
        }
        buffer_duty_cycle = false;
        buffer_lux = false;
        buffer_read_size = 0;
        buffer_read_counter = 0;
    }

    // delay the time left to reach the sampling_time
    final_time = micros();
    delay(sampling_time - ((final_time - initial_time) * pow(10, -3)));
}

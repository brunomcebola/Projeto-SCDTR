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
float duty_cycle = 0;
float lux = 0;
bool stream_duty_cycle = false;
bool stream_lux = false;
bool buffer_duty_cycle = false;
bool buffer_lux = false;
int buffer_read_size = 0;
int buffer_read_counter = 0;

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

    //
    interface();

    u_sim = simulator.simulate(micros());
    u_real = n_to_volt(analogRead(A0));

    controller.anti_wind_up();

    controller.calc_u_fb(u_sim, u_real);
    controller.calc_u_ff(simulator.get_gain());

    u_cont = controller.get_control_signal();

    analogWrite(LED_PIN, volt_to_n(u_cont));

    duty_cycle = u_cont / V_REF;

    lux = n_to_lux(analogRead(A0), simulator.get_gain());

    data = {duty_cycle, lux};

    buffer.insert_new(data);

    if (stream_duty_cycle) {
        Serial.printf("s d %c %f %d\n", lum, duty_cycle, millis());
    }

    if (stream_lux) {
        Serial.printf("s l %c %f %d\n", lum, lux, millis());
    }

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
        if(buffer_duty_cycle || buffer_lux) {
          Serial.println();
        }
        buffer_duty_cycle = false;
        buffer_lux = false;
        buffer_read_size = 0;
        buffer_read_counter = 0;
    }

    // delay the time left to reach the sampling_time
    final_time = micros();

    delay(sampling_time -
          ((final_time - initial_time) * pow(10, -3)));  // sampling time;
}

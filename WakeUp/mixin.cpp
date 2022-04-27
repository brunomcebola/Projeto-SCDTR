#include "mixin.h"

#include <math.h>

#include <algorithm>

#include "constants.h"

float n_to_volt(int n) { return (n * V_REF) / ANALOG_MAX; }

float n_to_lux(int n, float G) { return n * G; }

int volt_to_n(float v) { return (v * ANALOG_MAX) / V_REF; }

float ldr_volt_to_lux(float v, float m, float b) {
    return pow(10, (log10(get_ldr_resistance_from_v(v, m, b)) - b) / m);
}

int lux_to_n(float lux, float G) { return lux / G; }

float lux_to_volt(float lux, float G) { return n_to_volt(lux_to_n(lux, G)); }

float get_ldr_resistance_from_v(float v, float m, float b) {
    return ((R2 * V_REF) / v) - R2;
}

float get_ldr_resistance_from_lux(float lux, float m, float b) {
    return pow(10, m * log10(lux) + b);
}

float get_tau_for_n(int n) {
    return (
        0.389 + (8.1 * pow(10, -4) * pow(n, 1)) +
        (-1.86 * pow(10, -6) * pow(n, 2)) + (1.39 * pow(10, -9) * pow(n, 3)) +
        (-4.95 * pow(10, -13) * pow(n, 4)) + (8.58 * pow(10, -17) * pow(n, 5)) +
        (-5.82 * pow(10, -21) * pow(n, 6)));
}

int find_index(uint8_t arr[], int len, uint8_t v) {
    return distance(arr, find(arr, arr + len, v));
}
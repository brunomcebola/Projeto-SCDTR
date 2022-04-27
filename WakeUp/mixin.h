#ifndef MIXIN_H
#define MIXIN_H

#include <stdint.h>

float n_to_volt(int n);
float n_to_lux(int n, float G);
int volt_to_n(float v);
float ldr_volt_to_lux(float v, float m, float b);
int lux_to_n(float lux, float G);
float lux_to_volt(float lux, float G);
float get_ldr_resistance_from_v(float v, float m, float b);
float get_ldr_resistance_from_lux(float lux, float m, float b);
float get_tau_for_n(int n);

template <class T>
int find_index(T arr[], int len, T v);

#endif
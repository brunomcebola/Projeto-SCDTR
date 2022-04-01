#ifndef MIXIN_H
#define MIXIN_H

float n_to_volt(int n);

int volt_to_n(float v);

float get_ldr_resistance_from_v(float v, float m, float b);

float get_ldr_resistance_from_lux(float lux, float m, float b);

float volt_to_lux(float v, float m, float b);

float get_tau_for_n(int n);

#endif
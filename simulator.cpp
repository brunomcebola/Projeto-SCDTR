#include "simulator.h"

#include <math.h>

#include "constants.h"
#include "mixin.h"

using namespace std;

// private class functions

// public class functions

void Simulator::set_simulator(float m, float b, float G) {
    _m = m;
    _b = b;
    _G = G;
}

void Simulator::set_simualtion(int initial_time, int initial_level,
                               int target_level) {
    _t_0 = initial_time;

    _tau = get_tau_for_n(target_level);

    _v_i = n_to_volt(initial_level);

    _v_f = (V_REF * R2) /
           (R2 + get_ldr_resistance_from_lux(_G * target_level, _m, _b));
}

float Simulator::simulate(long int t) {
    return _v_f - (_v_f - _v_i) * exp(-((t - _t_0) * pow(10, -6) / _tau));
}

float Simulator::get_gain() {
    return _G;
}
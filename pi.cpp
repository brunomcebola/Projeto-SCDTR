#include "pi.h"

void Pi::set_pi(float T, float kp, float ki) {
    _up = 0;
    _ui = 0;
    _T = T;
    _kp = kp;
    _ki = ki;
}

float Pi::calc(float u_ref, float u) {
    float e = u_ref - u;
    _up = _kp * e;
    _ui = _ui + _ki * _T * e;

    return _up + _ui;
}

float Pi::get_up() { return _up; }

float Pi::get_ui() { return _ui; }
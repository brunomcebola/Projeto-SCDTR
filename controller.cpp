#include "controller.h"

#include "constants.h"
#include "mixin.h"

void Controller::set_lux_ref(float lux_ref) { _lux_ref = lux_ref; }

float Controller::get_lux_ref() { return _lux_ref; }

float Controller::get_control_signal(float u) {
    if (u > V_REF)
        return V_REF;
    else if (u < 0)
        return 0;
    else
        return u;
}

void Controller::set_controller(float T, float kp, float ki) {
    _u_fb = 0;
    _u_fb = 0;
    _lux_ref = 0;
    _use_fb = true;
    _use_ff = true;
    _pi.set_pi(T, kp, ki);
}

float Controller::calc_u_fb(float u_sim, float u_real) {
    if (_use_fb)
        _u_fb = _pi.calc(u_sim, u_real);
    else
        _u_fb = 0;

    return _u_fb;
}

float Controller::calc_u_ff(float G) {
    if (_use_ff)
        _u_ff = n_to_volt(_lux_ref / G);
    else
        _u_ff = 0;

    return _u_ff;
}
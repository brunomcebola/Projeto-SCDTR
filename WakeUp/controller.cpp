#include "controller.h"

#include "constants.h"
#include "mixin.h"

void Controller::set_lux_ref(float lux_ref) { _lux_ref = lux_ref; }

float Controller::get_lux_ref() { return _lux_ref; }

float Controller::get_control_signal(float u) {
    if (u > V_REF)
        _u = V_REF;
    else if (u < 0)
        _u = 0;
    else
        _u = u;

    return _u;
}

float Controller::get_control_signal() {
    return get_control_signal(_u_ff + _u_fb);
}

void Controller::anti_wind_up() {
    if (_use_awp) {
        if (_u_ff + _u_fb > V_REF) {
            _pi.set_ui(V_REF - _u_ff);
        } else if (_u_ff + _u_fb < 0) {
            _pi.set_ui(-_u_ff);
        }
    }
}

void Controller::set_controller(float T, float kp, float ki) {
    _u_fb = 0;
    _u_fb = 0;
    _lux_ref = 0;
    _use_fb = true;
    _use_ff = true;  
    _use_awp = true;
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

float Controller::get_u() { return _u; }

void Controller::set_ff_usage(bool sel) { _use_ff = sel; }

void Controller::set_fb_usage(bool sel) { _use_fb = sel; }

void Controller::set_anti_wind_up_usage(bool sel) { _use_awp = sel; }

bool Controller::get_ff_usage() { return _use_ff; }

bool Controller::get_fb_usage() { return _use_fb; }

bool Controller::get_anti_wind_up_usage() { return _use_awp; }

void Controller::set_occupancy(bool sel) { _occupied = sel; };

bool Controller::get_occupancy() { return _occupied; };
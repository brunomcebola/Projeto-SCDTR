#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pi.h"

class Controller {
   private:
    Pi _pi;
    float _lux_ref, _u_fb, _u_ff, _u;
    bool _use_fb, _use_ff, _use_awp, _occupied;

   public:
    void set_controller(float T, float kp, float ki);

    void set_lux_ref(float lux_ref);

    float get_lux_ref();

    float get_control_signal();
    float get_control_signal(float u);

    float calc_u_fb(float u_sim, float u_real);

    float calc_u_ff(float G);

    float get_u();

    void anti_wind_up();

    void set_ff_usage(bool sel);

    void set_fb_usage(bool sel);

    void set_anti_wind_up_usage(bool sel);

    bool get_ff_usage();

    bool get_fb_usage();

    bool get_anti_wind_up_usage();

    void set_occupancy(bool sel);

    bool get_occupancy();
};

#endif
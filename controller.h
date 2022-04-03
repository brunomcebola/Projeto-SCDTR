#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pi.h"

class Controller {
   private:
    Pi _pi;
    float _lux_ref, _u_fb, _u_ff;
    bool _use_fb, _use_ff;

   public:
    void set_controller(float T, float kp, float ki);
    void set_lux_ref(float lux_ref);
    float get_lux_ref();
    float get_control_signal(float u);
    float calc_u_fb(float u_sim, float u_real);
    float calc_u_ff(float G);
};

#endif
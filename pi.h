#ifndef PI_H
#define PI_H

class Pi {
   private:
    float _kp, _ki, _T, _up, _ui;

   public:
    void set_pi(float T, float kp, float ki);
    float calc(float u_ref, float u);
    float get_up();
    float get_ui();
};

#endif
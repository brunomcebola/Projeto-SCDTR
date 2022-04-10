#ifndef SIMULATOR_H
#define SIMULATOR_H

class Simulator {
   private:
    // internal members
    int _t_0;
    float _m, _b, _G, _tau, _v_i, _v_f;

   public:
    // exposed members
    void set_simulator(float m, float b, float G);
    void set_simualtion(int initial_time, int initial_level, int target_level);
    float simulate(long int t);
    float get_gain();
};

#endif
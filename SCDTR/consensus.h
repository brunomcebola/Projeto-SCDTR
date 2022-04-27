/*
  consensus.h - Library for consensus algorithm.
  Created by João Luzio in March 2022.
*/

#ifndef consensus_h
#define consensus_h

#define N_RPI 3

class consensus{

  public: 
    consensus(int N); // Constructor
    void defining(int id, float* K, float my_cost, float ext, float lum);
    bool check_feasibility(float* d);
    float evaluate_cost(float* d, float rho);
    float iterate(float rho, float iter);
    float vec_mult(float* vec1, float* vec2);
 
  private:
    static int n_rpi;
    int index;
    float d[N_RPI];
    float d_av[N_RPI];
    float y[N_RPI];
    float gain[N_RPI];
    float n;
    float m;
    float c[N_RPI];
    float o;
    float L;
    float best_cost;

};

#endif
/*
  consensus.h - Library for consensus algorithm.
  Created by João Luzio in March 2022.
*/

#ifndef consensus_h
#define consensus_h

#define N_RPI 3

class Consensus{

  public: 
    // Constructor
    Consensus(int N);
    void defining(int id, float* K, float my_cost, float ext, float lum, float r);

    // Accessors
    float get_d_av(int index);
    float get_d(int index);
    float get_y(int index);
    float get_boundary_occupied(void);
    float get_boundary_unoccupied(void);
    float get_gain(int index);
    float get_cost(void);
    
    // Modify
    void set_d_av(float val, int index);
    void set_y(float val, int index);
    void set_occupation(bool state);
    void set_uboundary(float val);
    void set_oboundary(float val);
    void set_cost(float val);

    // Consensus
    bool check_feasibility(float* d);
    float evaluate_cost(float* d);
    float iterate(void);
    float vec_mult(float* vec1, float* vec2);

 
  private:
    int n_rpi;
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
    float rho;
    bool occupied_flag;
    float unoccupied_lower_boundary;
    float occupied_lower_boundary;

};

#endif

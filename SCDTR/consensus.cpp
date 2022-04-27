#include <cmath>
#include "consensus.h"

// Class Constructor
consensus::consensus(int N){

  n_rpi = N;
  index = 0;
  rho = 0.0;
  for(int i = 0; i < n_rpi; i++){
    d[i] = 0.0;
    d_av[i] = 0.0;
    y[i] = 0.0;
    gain[i] = 0.0;
    c[i] = 0.0;
  }
  n = 0,0;
  m = 0.0;
  o = 0.0;
  L = 0.0;

  occupied_flag = false;
  unoccupied_lower_boundary = 50.0;
  occupied_lower_boundary = 0.0;

}

// Define/Redefine Values (Kind of a Reset)
void consensus::defining(int id, float* K, float my_cost, float ext, float lum, float r){

  index = id-1;
  rho = r;

  occupied_flag = true;
  unoccupied_lower_boundary = 0.0;
  occupied_lower_boundary = 0.0;

  for(int i = 0; i < n_rpi; i++){

    d[i] = 0.0;
    d_av[i] = 0.0;
    y[i] = 0.0;

    gain[i] = K[i];

    if(i == index) c[i] = my_cost;
    else c[i] = 0.0;

  }
  
  n = 0;
  for(int i = 0; i < n_rpi; i++ ) n += gain[i]*gain[i];
  m = n - gain[index]*gain[index];

  o = ext;
  L = lum;

}

// Cjeck if it is Feasible
bool consensus::check_feasibility(float* d){

    float tol = 0.001; // tolerance for rounding errors;
    float cond = 0.0;

    for(int i = 0; i < n_rpi; i++){
      cond += (d[i]*gain[i]);
    }

    if( d[index] < 0 - tol  ) return false;
    if( d[index] > 100 + tol) return false;
    if( cond < L - o - tol ) return false;

    return true;
}

// Evaluates the Cost
float consensus::evaluate_cost(float* d){

    float cost;
    float aux[n_rpi]{0.0};
    float multiplication = 0.0;
    float norm = 0.0;
    float acum = 0.0;

    for(int i = 0; i < n_rpi; i++){
      norm += (d[i] - d_av[i] )*(d[i] - d_av[i] );
      multiplication += c[i] * d[i];
      aux[i] = d[i] - d_av[i];
      acum += y[i]*aux[i];
    }
    norm = sqrt(norm);

    cost = multiplication + acum + (rho/2)*norm*norm;

    return cost;
}

// Consensus Algorithm Iteration
float consensus::iterate(void){

    float d_best[n_rpi]{-1};
    float cost_best{10000}; // large num
    int i;

    bool sol_unconstrained = true;
    bool sol_boundary_linear = true;
    bool sol_boundary_0 = true;
    bool sol_boundary_100 = true;
    bool sol_linear_0 = true;
    bool sol_linear_100 = true;

    float z[n_rpi];
    
    float d_bl[n_rpi]; // linear boundary
    float d_b0[n_rpi]; // lower boundary
    float d_b1[n_rpi]; // upper boundary

    float d_l0[n_rpi]; // minimum constrained to linear and 0 boundary
    float d_l1[n_rpi]; // minimum constrained to linear and 100 boundary

    // conditions start here
    float d_u[n_rpi];
    for(i = 0; i < n_rpi; i++){
      z[i] = d_av[i]*rho - y[i] - c[i];
      d_u[i] = z[i]/rho;
    }

    // unconstrained minimum
    sol_unconstrained = check_feasibility(d_u);
    if(sol_unconstrained){
      float cost_unconstrained = evaluate_cost(d_u);
      if(cost_unconstrained < cost_best){
        cost_best = cost_unconstrained;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_u[i];
      }
    }

    // compute minimum constrained to linear boundary
    for(int i = 0; i < n_rpi; i++){
      d_bl[i] = z[i]/rho - (gain[i]/n)*(o - L + (1/rho)*vec_mult(z, gain)) ;  
    }

    // check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(d_bl);

    // compute cost and if best store new optimum
    if(sol_boundary_linear){
      float cost_boundary_linear = evaluate_cost(d_bl);
      if(cost_boundary_linear < cost_best){
        cost_best = cost_boundary_linear;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_bl[i];
      }
    }

    // compute minimum constrained to 0 boundary
    for(int i = 0; i < n_rpi; i++) d_b0[i] = z[i]/rho;
    d_b0[index] = 0.0;

    // check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(d_b0);

    // compute cost and if best store new optimum
    if(sol_boundary_0){
      float cost_boundary_0 = evaluate_cost(d_b0);
      if(cost_boundary_0 < cost_best){
        cost_best = cost_boundary_0;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_b0[i];
      }
    }

    // compute minimum constrained to 100 boundary
    for(int i = 0; i < n_rpi; i++) d_b1[i] = z[i]/rho;
    d_b1[index] = 100.0;
    // check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(d_b1);

    // compute cost and if best store new optimum
    if(sol_boundary_100){
      float cost_boundary_100 = evaluate_cost(d_b1);
      if(cost_boundary_100 < cost_best){
        cost_best = cost_boundary_100;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_b1[i];
      }
    }

    // compute minimum constrained to linear and 0 boundary
    for(int i = 0; i < n_rpi; i++){
       d_l0[i] = z[i]/rho - (gain[i]/m)*(o - L) + (gain[i]/(rho*m))*(gain[index]*z[index] - vec_mult(z, gain));
    }
    if(occupied_flag == true) d_l0[index] = occupied_lower_boundary;
    else d_l0[index] = unoccupied_lower_boundary;

    // check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(d_l0);

    // compute cost and if best store new optimum
    if(sol_linear_0){
      float cost_linear_0 = evaluate_cost(d_l0);
      if(cost_linear_0 < cost_best){
        cost_best = cost_linear_0;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_l0[i];
      }
    }

    // compute minimum constrained to linear and 100 boundary
    for(int i = 0; i < n_rpi; i++){
       d_l1[i] = z[i]/rho - (gain[i]/m)*(o - L + 100*gain[index]) + (gain[i]/(rho*m))*(gain[index]*z[index] - vec_mult(z, gain));
    }
    d_l1[index] = 100.0;

    // check feasibility of minimum constrained to linear and 100 boundary
    sol_linear_100 = check_feasibility(d_l1);

    // compute cost and if best store new optimum
    if(sol_linear_100){
      float cost_linear_100 = evaluate_cost(d_l1);
      if(cost_linear_100 < cost_best){
        cost_best = cost_linear_100;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_l1[i];
      }
    }

    // final results
    for(i = 0; i < n_rpi; i++) d[i] = d_best[i];

    best_cost = cost_best;

    return cost_best;
    
}

// Multiply two vectors (dot product)
float consensus::vec_mult(float* vec1, float* vec2){

    float mult = 0.0;

    for(int i = 0; i < n_rpi; i++){
      mult += vec1[i]*vec2[i];
    }

    return mult;

}

// Accessors
float consensus::get_d_av(int index) { return d_av[index]; }
float consensus::get_d(int index) { return d[index]; }
float consensus::get_y(int index) { return y[index]; }
float consensus::get_gain(int index) { return gain[index]; }
float consensus::get_boundary_occupied(void) { return occupied_lower_boundary; }
float consensus::get_boundary_unoccupied(void) { return unoccupied_lower_boundary; }

// Modify
void consensus::set_d_av(float val, int index) { d_av[index] = val; }
void consensus::set_y(float val, int index) { y[index] = val; }
void consensus::set_occupation(bool state) { occupied_flag = state; }
void consensus::set_uboundary(float val){
  if(val > occupied_lower_boundary) unoccupied_lower_boundary = occupied_lower_boundary;
  else if(val < 0.0) unoccupied_lower_boundary = 0.0;
  else if(val > 100.0) unoccupied_lower_boundary = 100.0;
  else unoccupied_lower_boundary = val;
}
void consensus::set_oboundary(float val){
  if(val < occupied_lower_boundary) occupied_lower_boundary = unoccupied_lower_boundary;
  else if(val < 0.0) occupied_lower_boundary = 0.0;
  else if(val > 100.0) occupied_lower_boundary = 100.0;
  else occupied_lower_boundary = val;
}

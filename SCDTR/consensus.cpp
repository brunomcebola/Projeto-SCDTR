#include <cmath>
#include <iostream>
#include "consensus.h"

/*
float cost{0};
float d[2]{0.0,0.0};

//case 1
float L1 = 80, L2 = 150;
float o1 = 50, o2 = 50;
float c1 = 1, c2=1;
float rho =0.07;
int maxiter = 50;
float k11= 2, k12=0.5, k21=0.5, k22=2;

float K[2][2]{k12, k12, k21,k22};
float c[2]{c1,c2};
float L[2]{L1,L2};
float o[2]{o1,o2}; 

//check if it is going well
float d11[50]{0};
float d12[50]{0};
float d21[50]{0};
float d22[50]{0};

float av1[50]{0};
float av2[50]{0};
*/

/*
int main(){
    consensus_node node1; 
    node1.index = 0;
    for(int i = 0; i < 2; i++){
      node1.d[i] = 0.0;
      node1.d_av[i] = 0.0;
      node1.y[i] = 0.0;
    }
    node1.gain[0] = k11;
    node1.gain[1] = k12;
    node1.n = k11*k11 + k12*k12;
    node1.m = k12*k12;
    node1.c[0] = c1;
    node1.c[1] = 0.0;
    node1.o = o1;
    node1.L = L1;
    
    consensus_node node2;
    node2.index = 1;
    for(int i = 0; i < 2; i++){
      node2.d[i] = 0.0;
      node2.d_av[i] = 0.0;
      node2.y[i] = 0.0;
    }
    node2.gain[0] = k21;
    node2.gain[1] = k22;
    node2.n = k22*k22 + k21*k21;
    node2.m = k21*k21;
    node2.c[0] = 0.0;
    node2.c[1] = c2;
    node2.o = o2;
    node2.L = L2;

    for(int i = 1; i < maxiter; i++){
        iterate(&node1, 0.07,i);
        iterate(&node2, 0.07,i);
        
        node1.d_av[0] = (node1.d[0] + node2.d[0])/2;
        node1.d_av[1] = (node1.d[1] + node2.d[1])/2;

        node2.d_av[0] = (node1.d[0] + node2.d[0])/2;
        node2.d_av[1] = (node1.d[1] + node2.d[1])/2;
        
        node1.y[0] = node1.y[0] + rho*(node1.d[0]-node1.d_av[0]);
        node1.y[1] = node1.y[1] + rho*(node1.d[1]-node1.d_av[1]);

        node2.y[0] = node2.y[0] + rho*(node2.d[0]-node2.d_av[0]);
        node2.y[1] = node2.y[1] + rho*(node2.d[1]-node2.d_av[1]);

        d11[i] = node1.d[0];
        d12[i] = node1.d[1];
        d21[i] = node2.d[0];
        d22[i] = node2.d[1];

        av1[i] = (d11[i]+d21[i])/2;
        av2[i] = (d12[i]+d22[i])/2;
    }

    return 0;
}
*/

// Class Constructor
consensus::consensus(int N){

  n_rpi = N;
  index = 0;
  for(int i = 0; i < n_rpi; i++){
    d[i] = 0.0;
    d_av[i] = 0.0;
    y[i] = 0.0;
    gain[i] = 0.0;
    c[i] = 0.0;
  }
  n = 0;
  m = 0;
  o = 0.0;
  L = 0.0;

}

// Define/Redefine Values
void consensus::defining(int id, float* K, float my_cost, float ext, float lum){

  index = id-1;

  for(int i = 0; i < n_rpi; i++){

    d[i] = 0.0;
    d_av[i] = 0.0;
    y[i] = 0.0;

    gain[i] = K[i];

    if(i == index) c[i] = my_cost;
    else c[i] = 0.0;

  }

  n = 0; // TODO
  m = 0; // TODO

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
float consensus::evaluate_cost(float* d, float rho){

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
float consensus::iterate(float rho, float iter){

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
      z[i] = d_av[i] * rho - y[i] - c[i];
      d_u[i] = z[i] / rho;
    }

    // unconstrained minimum
    sol_unconstrained = check_feasibility(d_u);
    if(sol_unconstrained){
      float cost_unconstrained = evaluate_cost(d_u, rho);
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
      float cost_boundary_linear = evaluate_cost(d_bl, rho);
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
      float cost_boundary_0 = evaluate_cost(d_b0, rho);
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
      float cost_boundary_100 = evaluate_cost(d_b1, rho);
      if(cost_boundary_100 < cost_best){
        cost_best = cost_boundary_100;
        for(i = 0; i < n_rpi; i++) d_best[i] = d_b1[i];
      }
    }

    // compute minimum constrained to linear and 0 boundary
    for(int i = 0; i < n_rpi; i++){
       d_l0[i] = z[i]/rho - (gain[i]/m)*(o - L) + (gain[i]/(rho*m))*(gain[index]*z[index] - vec_mult(z, gain));
    }
    d_l0[index] = 0.0;

    // check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(d_l0);

    // compute cost and if best store new optimum
    if(sol_linear_0){
      float cost_linear_0 = evaluate_cost(d_l0, rho);
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
      float cost_linear_100 = evaluate_cost(d_l1, rho);
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

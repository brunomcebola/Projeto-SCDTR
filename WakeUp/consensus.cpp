#include <cmath>
#include <iostream>

using namespace std;

/*
 * gcc consensus.cpp -lstdc++ -o consensus -lm
 * ./consensus
 */

struct consensus_node{
    int index;
    float d[2];
    float d_av[2];
    float y[2];
    float gain[2];
    float n; // norm(node.k)^2
    float m;
    float c[2];
    float o;
    float L;
    /*
    consensus_node(int _index, float _d[], float _d_av[], float _y[],
    float _gain[], float _n, float _m, float _cost[], float _o, float _L)
    : index{_index},d{*_d}, d_av{*_d_av}, y{*_y}, gain{*_gain},n{_n},m{_m},
    c{*_cost}, o{_o}, L{_L} {}
    */
};

float cost{0};
float d[2]{0.0,0.0};

//case 1
float L1 = 150, L2 = 80;
float o1 = 30, o2 = 0;
float c1 = 1, c2=1;
float rho =0.07;
int maxiter = 50;
float k11= 2, k12=0.5, k21=0.5, k22=2;

float K[2][2]{k12, k12, k21,k22};
float c[2]{c1,c2};
float L[2]{L1,L2};
float o[2]{o1,o2}; 

//check if it is going well
float d11[50];
float d12[50];
float d21[50];
float d22[50];

float av1[50];
float av2[50];

bool check_feasibility(consensus_node *node,float d[]);
float evaluate_cost(consensus_node *node, float d[], float rho);
void consensus_iterate(consensus_node *node, float rho);

float vec_1_by_2_mult(float vec1[], float vec2[]){
    return vec1[0]*vec2[0] + vec1[1]*vec2[1];
}

float * vec_2_by_1_mult(float vec1[], float vec2[]){
    float *new_vec = {0};

    new_vec[0] = vec1[0] * vec2[0];
    new_vec[1] = vec1[1] * vec2[1];

    return new_vec;
}

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

    for(int i = 0; i < maxiter; i++){
        consensus_iterate(&node1, 0.001);
        consensus_iterate(&node2, 0.001);

        //change cuz of arrays;
        node1.d_av[0] = (node1.d[0] + node2.d[0])/2;
        node1.d_av[1] = (node1.d[0] + node2.d[1])/2;

        node2.d_av[0] = (node1.d[0] + node2.d[0])/2;
        node2.d_av[1] = (node1.d[0] + node2.d[1])/2;

        //change cuz of arrays
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

    cout << d[0] << endl;
    cout << d[1] << endl;
    cout << cost << endl;

    return 0;
}

bool check_feasibility(consensus_node *node,float d[]){
    float tol = 0.001; // tolerance for rounding errors;

    if( d[(*node).index] < 0 - tol  ) return false;
    if( d[(*node).index] > 100 + tol) return false;

    if( d[0] * (*node).gain[0] + d[1] * (*node).gain[1]< (*node).L - (*node).o -tol ) return false;

    return true;
}

float evaluate_cost(consensus_node *node, float d[], float rho){

    float cost;
    float aux[2];
    float multiplication;

    float norm = sqrt( (d[0] - (*node).d_av[0] )*(d[0] - (*node).d_av[0] ) + (d[1] - (*node).d_av[1] )*(d[1] - (*node).d_av[1] ) );

    multiplication = (*node).c[0] * d[0] + (*node).c[1] * d[1]; 
    aux[0] = d[0] - (*node).d_av[0];
    aux[1] = d[1] - (*node).d_av[1];
    cost = multiplication + ( (*node).y[0]*aux[0] + (*node).y[1]*aux[1] ) + (rho/2) * norm * norm;

    return cost;
}

void consensus_iterate(consensus_node *node, float rho){
    float d_best[2]{-1,-1};
    float cost_best{10000}; // large num

    bool sol_unconstrained = true;
    bool sol_boundary_linear = true;
    bool sol_boundary_0 = true;
    bool sol_boundary_100 = true;
    bool sol_linear_0 = true;
    bool sol_linear_100 = true;


    float z[2];
    
    float d_bl[2]; // linear boundary
    float d_b0[2]; // lower boundary
    float d_b1[2]; // upper boundary

    float d_l0[2]; // minimum constrained to linear and 0 boundary
    float d_l1[2]; // minimum constrained to linear and 100 boundary

    z[0] = rho*(*node).d_av[0] - (*node).y[0] - (*node).c[0];
    z[1] = rho*(*node).d_av[1] - (*node).y[1] - (*node).c[1];

    // CONDITIONS START HERE!!!!
    float d_u[2];
    d_u[0] = z[0] / rho;
    d_u[1] = z[1] / rho;
    // unconstrained minimum
    sol_unconstrained = check_feasibility(node,d_u);
    if(sol_unconstrained){
      float cost_unconstrained = evaluate_cost(node, d_u, rho);
      if(cost_unconstrained < cost_best){
        cost_best = cost_unconstrained;
        d_best[0] = d_u[0];
        d_best[1] = d_u[1];
      }
    }
    // compute minimum constrained to linear boundary
    for(int i = 0; i < 2; i++){
      d_bl[i] = z[i]/rho - ((*node).gain[i]/(*node).n)*((*node).o - (*node).L + (1/rho)*vec_1_by_2_mult(z, (*node).gain)) ;  
    }
    // check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(node, d_bl);
    // compute cost and if best store new optimum
    if(sol_boundary_linear){
      float cost_boundary_linear = evaluate_cost(node, d_bl, rho);
      if(cost_boundary_linear < cost_best){
        cost_best = cost_boundary_linear;
        d_best[0] = d_bl[0];
        d_best[1] = d_bl[1];
      }
    }
    // compute minimum constrained to 0 boundary
    for(int i = 0; i < 2; i++) d_b0[i] = z[i]/rho;
    d_b0[(*node).index] = 0.0;
    // check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(node, d_b0);
    // compute cost and if best store new optimum
    if(sol_boundary_0){
      float cost_boundary_0 = evaluate_cost(node, d_b0, rho);
      if(cost_boundary_0 < cost_best){
        cost_best = cost_boundary_0;
        d_best[0] = d_b0[0];
        d_best[1] = d_b0[1];
      }
    }
    // compute minimum constrained to 100 boundary
    for(int i = 0; i < 2; i++) d_b1[i] = z[i]/rho;
    d_b1[(*node).index] = 0.0;
    // check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(node, d_b1);
    // compute cost and if best store new optimum
    if(sol_boundary_100){
      float cost_boundary_100 = evaluate_cost(node, d_b1, rho);
      if(cost_boundary_100 < cost_best){
        cost_best = cost_boundary_100;
        d_best[0] = d_b1[0];
        d_best[1] = d_b1[1];
      }
    }
    // compute minimum constrained to linear and 0 boundary
    for(int i = 0; i < 2; i++){
       d_l0[i] = z[i]/rho - ((*node).gain[i]/(*node).n)*((*node).o - (*node).L) + ((*node).gain[i]/(rho*(*node).m))*((*node).gain[(*node).index]*z[(*node).index] - vec_1_by_2_mult(z, (*node).gain));
    }
    d_l0[(*node).index] = 0.0;
    // check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, d_l0);
    // compute cost and if best store new optimum
    if(sol_linear_0){
      float cost_linear_0 = evaluate_cost(node, d_l0, rho);
      if(cost_linear_0 < cost_best){
        cost_best = cost_linear_0;
        d_best[0] = d_l0[0];
        d_best[1] = d_l0[1];
      }
    }
    // compute minimum constrained to linear and 100 boundary
    for(int i = 0; i < 2; i++){
       d_l1[i] = z[i]/rho - ((*node).gain[i]/(*node).n)*((*node).o - (*node).L + 100*(*node).gain[(*node).index]) + ((*node).gain[i]/(rho*(*node).m))*((*node).gain[(*node).index]*z[(*node).index] - vec_1_by_2_mult(z, (*node).gain));
    }
    d_l1[(*node).index] = 100.0;
    // check feasibility of minimum constrained to linear and 100 boundary
    sol_linear_100 = check_feasibility(node, d_l1);
    // compute cost and if best store new optimum
    if(sol_linear_100){
      float cost_linear_100 = evaluate_cost(node, d_l1, rho);
      if(cost_linear_100 < cost_best){
        cost_best = cost_linear_100;
        d_best[0] = d_l1[0];
        d_best[1] = d_l1[1];
      }
    }
    //need something here
    d[0] = d_best[0];
    d[1] = d_best[1];
    cost = cost_best;
}

#include <cmath>
/*
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
    consensus_node(int _index, float _d[], float _d_av[], float _y[],
    float _gain[], float _n, float _m, float _cost[], float _o, float _L)
    : index{_index},d{*_d}, d_av{*_d_av}, y{*_y}, gain{*_gain},n{_n},m{_m},
    c{*_cost}, o{_o}, L{_L} {}
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

float vec_1_by_2_mult(float vec1[], float vec2[]){
    return vec1[0]*vec2[0] + vec1[1]*vec2[1];
}

float * vec_2_by_1_mult(float vec1[], float vec2[]){
    float new_vec[2];

    new_vec[0] = vec1[0] * vec2[0];
    new_vec[1] = vec1[1] * vec2[1];

    return new_vec;
}

int main(){
    consensus_node node1 = {1, {0,0}, {0,0},{0,0},{k11,k12}, k11*k11 + k12*k12, k12*k12, {c1,0}, o1,L1}; 
    consensus_node node2 = {1, {0,0}, {0,0},{0,0},{k21,k22}, k21*k21 + k22*k22, k21*k21, {0,c2}, o2,L2};

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
}

bool check_feasibility(consensus_node *node,float d[]){
    float tol = 0.001; // tolerance for rounding errors;

    if( d[(*node).index] < 0 - tol  ) return false;
    if( d[(*node).index] > 100 + tol) return false;

    if( d[0] * (*node).gain[0] + d[1] * (*node).gain[1]< (*node).L - (*node).o -tol )
        return false;

    return true;
}

float evaluate_cost(consensus_node *node, float d[], float rho){

    float cost;
    float aux[2];
    float multiplication;

    float norm = sqrt( (d[0] - (*node).d_av[0] )*(d[0] - (*node).d_av[0] )  + 
            (d[1] - (*node).d_av[1] )*(d[1] - (*node).d_av[1] ) );

    multiplication = (*node).c[0] * d[0] + (*node).c[1] * d[1]; 
    aux[0] = d[0] - (*node).d_av[0];
    aux[1] = d[1] - (*node).d_av[1];
    cost = multiplication + ( (*node).y[0]*aux[0] + (*node).y[1]*aux[1] ) +
    (rho/2) * norm * norm;

    return cost;
}

void consensus_iterate(consensus_node *node, float rho){
    float d_best[2]{-1,-1};
    float cost_best{10000};

    bool sol_unconstrained = true;
    bool sol_boundary_linear = true;
    bool sol_boundary_0 = true;
    bool sol_boundary_100 = true;
    bool sol_linear_0 = true;
    bool sol_linear_100 = true;


    float z[2];

    z[0] = rho*(*node).d_av[0] - (*node).y[0] - (*node).c[0];
    z[1] = rho*(*node).d_av[1] - (*node).y[1] - (*node).c[1];



    // CONDITIONS START HERE!!!!

    float d_u[2];
    d_u[0] = z[0] / rho;
    d_u[1] = z[1] / rho;

    bool sol_unconstrained = check_feasibility(node,d_u);
    if(sol_unconstrained){
        float cost_unconstrained = evaluate_cost(node, d_u, rho);
        if(cost_unconstrained < cost_best){
            cost_best = cost_unconstrained;
            d_best[0] = d_u[0];
            d_best[1] = d_u[1];
        }
    }

    //need something here
    d[0] = d_best[0];
    d[1] = d_best[1];
    cost = cost_best;
}
*/

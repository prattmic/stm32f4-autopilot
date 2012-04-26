#include "trim.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#define MAXLINE 100

int getnewline(char line[], int max);

int main(void) {
    char line[MAXLINE];
    float alpha, beta, phi, Va, R, gamma;
    struct plane UAV = { 0 };
    struct trim_out result;
    UAV.m =             1.56;
    UAV.J.x =           0.1147;
    UAV.J.y =           0.0576;
    UAV.J.z =           0.1712;
    UAV.J.xz =          0.0015;
    UAV.S =             0.2589;
    UAV.b =             1.4224;
    UAV.c =             0.3302;
    UAV.S_prop =        0.0314;
    UAV.row =           1.2682;
    UAV.k_motor =       80;
    UAV.k_T_p =         0;
    UAV.k_omega =       0;
    UAV.C_L.zero =      0.28;
    UAV.C_D.zero =      0.03;
    UAV.C_m.zero =      -0.02338;
    UAV.C_L.alpha =     3.45;
    UAV.C_D.alpha =     0.3;
    UAV.C_m.alpha =     -0.38;
    UAV.C_L.q =         0;
    UAV.C_D.q =         0;
    UAV.C_m.q =         -3.6;
    UAV.C_L.delta_e =   -0.36;
    UAV.C_D.delta_e =   0;
    UAV.C_m.delta_e =   -0.5;
    UAV.C_Y.zero =      0;
    UAV.C_l.zero =      0;
    UAV.C_n.zero =      0;
    UAV.C_Y.beta =      -0.07359;
    UAV.C_l.beta =      -0.02854;
    UAV.C_n.beta =      -0.0004;
    UAV.C_Y.p =         0;
    UAV.C_l.p =         -0.3209;
    UAV.C_n.p =         -0.01297;
    UAV.C_Y.r =         0;
    UAV.C_l.r =         0.03066;
    UAV.C_n.r =         -0.00434;
    UAV.C_Y.delta_a =   0;
    UAV.C_l.delta_a =   0.1682;
    UAV.C_n.delta_a =   -0.00328;
    UAV.C_Y.delta_r =   -0.17;
    UAV.C_l.delta_r =   0.105;
    UAV.C_n.delta_r =   -0.032;
    UAV.C_prop =        1;
    UAV.M =             50;
    UAV.alpha_0 =       0.4712;
    UAV.epsilon =       0.1592;
    UAV.g =             9.80665;

    compute_params(&UAV);

    while (1) {
        printf("alpha = ");
        if (scanf("%f", &alpha) != 1) {
            printf("error\n");
        }
        printf("beta = ");
        scanf("%f", &beta);
        printf("phi = ");
        scanf("%f", &phi);
        printf("Va = ");
        scanf("%f", &Va);
        printf("R = ");
        scanf("%f", &R);
        printf("gamma = ");
        scanf("%f", &gamma);

        result = minimize_J(alpha, beta, phi, Va, R, gamma, &UAV);
        //printf("J = %f\n", result);
        printf("alpha = %f beta = %f phi = %f i = %d\n", result.alpha, result.beta, result.phi, result.i);
    }

    return 0;
}

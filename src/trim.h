/* All the shit necessary for calculating trim values.  My God... */

#ifndef TRIM_H_INCLUDED
#define TRIM_H_INCLUDED

#include <math.h>
#include "matrix.h"

/* Oh, the variables!
 * Kill me now. */

struct trim_state {
    float u;            /* Inertial velocity projected onto bodu frame x-axis */
    float v;            /* Inertial velocity projected onto body frame y-axis */
    float w;            /* Inertial velocity projected onto body frame z-axis */
    float theta;        /* Pitch angle */
    float p;            /* Roll rate along body frame x-axis */
    float q;            /* Pitch rate along body frame y-axis */
    float r;            /* Yaw rate along body frame z-axis */
};

struct trim_input {
    float delta_e;      /* Elevator deflection */
    float delta_t;      /* Throttle deflection */
    float delta_a;      /* Aileron deflection */
    float delta_r;      /* Rudder deflection */
};

struct trim_out {
    float alpha;        /* Angle of attack */
    float phi;          /* Roll angle */
    float beta;         /* Sideslip angle */
};

struct dx {
    float dh;
    float du;
    float dv;
    float dw;
    float dphi;
    float dtheta;
    float dpsi;
    float dp;
    float dq;
    float dr;
};

/* Inertial Matix (Moments of Inertia) - See pg. 36 */
struct inertia {
    float x;
    float y;
    float z;
    float xz;
};

/* Interial parameters - Eq. 3.13, pg. 38 */
struct inertial_param {
    float Gamma;
    float one;
    float two;
    float three;
    float four;
    float five;
    float six;
    float seven;
    float eight;
};

/* Longitudinal Coefficient - See Ch. 4 or Appen. D */
struct coeff_lon {
    float zero;
    float alpha;
    float q;
    float delta_e;
};

/* Lateral Coefficient */
struct coeff_lat {
    float zero;
    float beta;
    float p;
    float r;
    float delta_a;
    float delta_r;
};

/* Physical constants from the plane */
struct plane {
    /* User-defined parameters */
    float m;                /* Mass */
    struct inertia J;       /* Moments of inertia */
    float S;                /* Wing surface area */
    float b;                /* Wing span */
    float c;                /* Mean cord length of wing */
    float S_prop;           /* Area of prop */
    float row;              /* Air density */
    float k_motor;          /* Motor efficiency */
    float k_T_p;
    float k_omega;
    struct coeff_lon C_L;   /* Lift Coefficient */
    struct coeff_lon C_D;   /* Drag Coefficient */
    struct coeff_lon C_m;   /* Pitching moment coefficient */
    struct coeff_lat C_Y;   /* Force Coefficient along body frame y-axis */
    struct coeff_lat C_l;   /* Moment Coefficient along body frame x-axis */
    struct coeff_lat C_n;   /* Moment Coefficient along body frame z-axis */
    float C_prop;           /* Aerodynamic Coefficient for the prop */
    float M;                /* Constant related to stall (pg. 49) */
    float alpha_0;          /* Constant related to stall (pg. 49) */
    float epsilon;
    float g;                /* Gravitational constant */
    /* Computed from above variables */
    struct inertial_param Gamma;   /* Inertial parameters used in calculations */
    struct coeff_lat C_p;
    struct coeff_lat C_r;
};


void compute_params(struct plane *P);
float compute_J(float alpha, float beta, float phi, float Va, float R, float gamma, struct plane *P);

#endif /* TRIM_H_INCLUDED */

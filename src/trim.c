/* Calculate Trim Values for various aspects of the plane.
 * See Section 5.3.3 for algorithms */

#include <math.h>

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
struct interia {
    float x;
    float y;
    float z;
    float xz;
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
}

/* Physical constants from the plane */
struct plane {
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
};


struct trim_out compute_J(float alpha, float beta, float phi, float Va, float R, float gamma, struct plane *P);

/* This is all for computing trims for a constant climb rate */

/* Algorithm 2 - pg. 71 */
struct trim_out compute_J(float alpha, float beta, float phi, float Va, float R, float gamma, struct plane *P) {
    struct dx xstar = { Va * sin(gamma),                    /* dh */
                        0,                                  /* du */
                        0,                                  /* dv */
                        0,                                  /* dw */
                        0,                                  /* dphi */
                        0,                                  /* dtheta */
                        Va/R,                               /* dpsi */
                        0,                                  /* dp */
                        0,                                  /* dq */
                        0 };                                /* dr */

    struct trim_state state = { Va * cos(alpha) * cos(beta),        /* u */
                                Va * sin(beta),                     /* v */
                                Va * sin(alpha) * cos(beta),        /* w */
                                alpha + gamma,                      /* theta */
                                -Va/R * sin(state.theta),           /* p */
                                Va/R * sin(phi) * cos(state.theta), /* q */
                                Va/R * cos(phi) * cos(state.theta)};/* r */

    struct trom_input input;

    /* Eq. 5.18 - Elevator Deflection */
    input.delta_e = ( (P->J.xz * (state.p^2 - state.r^2) + (P->J.x - P->J.z) * state.p * state.r)/(0.5 * P->row * Va^2 * P->c * P->S) - P->C_m.zero - (P->C_m.alpha * alpha) - P->C_m.q * (P->c * state.q / 2 * Va) ) / P->C_m.delta_e
}

#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED

struct pid {
    volatile float kp;                   //-----------------------------------------------------------------------------------
    volatile float kd;                   //PID constants. let the tuning... BEGIN! These will need to be loaded from NV memory
    volatile float ki;                   //-----------------------------------------------------------------------------------
    volatile float i;                    //integrator
    volatile float d;                    //differentiator
    volatile float windup_upperlimit;
    volatile float windup_lowerlimit;
    volatile float output_limit;
    volatile float past_error;           //E[n-1]
    volatile float tau;                  //derivative time constant for this PID
};

struct lateral_gains {
    volatile float kp_phi;
    volatile float kd_phi;
    volatile float kp_chi;
    volatile float ki_chi;
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
    float rho;              /* Air density */
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
    float aileron_limit;    /* Max aileron deflection */
    float elevator_limit;   /* Max elevator deflection */
    /* Computed from above variables */
    struct inertial_param Gamma;   /* Inertial parameters used in calculations */
    struct coeff_lat C_p;
    struct coeff_lat C_r;
};

float limit(float in, float l, float u);
float eval_pid(struct pid* this_pid, float set, float current, float Ts);
void compute_params(struct plane *P);

#endif /* PID_H_INCLUDED */

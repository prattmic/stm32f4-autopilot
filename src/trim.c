/* Calculate Trim Values for various aspects of the plane.
 * See Section 5.3.3 for algorithms */

#include "trim.h"

/* Computes extra plane parameters based on the user defined ones. */
void compute_params(struct plane *P) {
    /* Eq. 3.13 */
    P->Gamma.Gamma =    P->J.x*P->J.z - pow(P->J.xz,2);
    P->Gamma.one =      (P->J.xz*(P->J.x - P->J.y + P->J.z))/P->Gamma.Gamma;
    P->Gamma.two =      (P->J.z*(P->J.z - P->J.y) + pow(P->J.xz,2))/P->Gamma.Gamma;
    P->Gamma.three =    P->J.z/P->Gamma.Gamma;
    P->Gamma.four =     P->J.xz/P->Gamma.Gamma;
    P->Gamma.five =     (P->J.z - P->J.x)/P->J.y;
    P->Gamma.six =      P->J.xz/P->J.y;
    P->Gamma.seven =    (P->J.x*(P->J.x - P->J.y) + pow(P->J.xz,2))/P->Gamma.Gamma;
    P->Gamma.eight =    P->J.x/P->Gamma.Gamma;

    /* pg. 64 */
    P->C_p.zero =       P->Gamma.three * P->C_l.zero + P->Gamma.four * P->C_n.zero;
    P->C_p.beta =       P->Gamma.three * P->C_l.beta + P->Gamma.four * P->C_n.beta;
    P->C_p.p =          P->Gamma.three * P->C_l.p + P->Gamma.four * P->C_n.p;
    P->C_p.r =          P->Gamma.three * P->C_l.r + P->Gamma.four * P->C_n.r;
    P->C_p.delta_a =    P->Gamma.three * P->C_l.delta_a + P->Gamma.four * P->C_n.delta_a;
    P->C_p.delta_r =    P->Gamma.three * P->C_l.delta_r + P->Gamma.four * P->C_n.delta_r;

    P->C_r.zero =       P->Gamma.four * P->C_l.zero + P->Gamma.eight * P->C_n.zero;
    P->C_r.beta =       P->Gamma.four * P->C_l.beta + P->Gamma.eight * P->C_n.beta;
    P->C_r.p =          P->Gamma.four * P->C_l.p + P->Gamma.eight * P->C_n.p;
    P->C_r.r =          P->Gamma.four * P->C_l.r + P->Gamma.eight * P->C_n.r;
    P->C_r.delta_a =    P->Gamma.four * P->C_l.delta_a + P->Gamma.eight * P->C_n.delta_a;
    P->C_r.delta_r =    P->Gamma.four * P->C_l.delta_r + P->Gamma.eight * P->C_n.delta_r;
}

/* This is all for computing trims for a constant climb rate */

/* Algorithm 2 - pg. 71 */
float compute_J(float alpha, float beta, float phi, float Va, float R, float gamma, struct plane *P) {
    /* Copying formulas from a book is fun! */
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

    /* This is where the deflections we are solving for go */
    struct trim_input input;

    /* Our computed delta_a and delta_r go here */
    float da_dr_data[] = {0, 
                          0};
    struct matrix da_dr = { da_dr_data, 2, 1 }; 

    /* We multiply two matrices to find them (Eq. 5.20) */
    float da_dr_mult1_data[] = { P->C_p.delta_a, P->C_p.delta_r,
                                      P->C_r.delta_a, P->C_r.delta_r};
    struct matrix da_dr_mult1 = { da_dr_mult1_data, 2, 2 };

    float da_dr_mult2_data[] = { (-P->Gamma.one*state.p*state.r + P->Gamma.two*state.q*state.r)/(0.5*P->row*pow(Va,2)*P->S*P->b) - P->C_p.zero - P->C_p.beta*beta - P->C_p.p*(P->b*state.p)/(2*Va) - P->C_p.r*(P->b*state.r)/(2*Va),
                               (-P->Gamma.seven*state.p*state.r + P->Gamma.one*state.q*state.r)/(0.5*P->row*pow(Va,2)*P->S*P->b) - P->C_r.zero - P->C_r.beta*beta - P->C_r.p*(P->b*state.p)/(2*Va) - P->C_r.r*(P->b*state.r)/(2*Va) };
    struct matrix da_dr_mult2 = { da_dr_mult2_data, 2, 1 };

    /* Used in Eq. 5.19 and 5.10 */
    float C_D_alpha = P->C_D.zero + P->C_D.alpha * alpha;
    float C_L_alpha = P->C_L.zero + P->C_L.alpha * alpha;
    float C_X_alpha = -C_D_alpha*cos(alpha) + C_L_alpha*sin(alpha);
    float C_X_q_alpha = -P->C_D.q*cos(alpha) + P->C_L.q*sin(alpha);
    float C_X_delta_e_alpha = -P->C_D.delta_e*cos(alpha) + P->C_L.delta_e*sin(alpha);
    float C_Z_alpha = -C_D_alpha*sin(alpha) - C_L_alpha*cos(alpha);
    float C_Z_q_alpha = -P->C_D.q*sin(alpha) - P->C_L.q*cos(alpha);
    float C_Z_delta_e_alpha = -P->C_D.delta_e*sin(alpha) - P->C_L.delta_e*cos(alpha);

    /* This is f(x,u), which comes from Eq. 5.3-5.12.  We need all the deflection crap first */
    struct dx f_x_u;

    /* Eq. 5.18 - Elevator Deflection */
    input.delta_e = ( (P->J.xz * (pow(state.p,2) - pow(state.r,2)) + (P->J.x - P->J.z) * state.p * state.r)/(0.5 * P->row * pow(Va,2) * P->c * P->S) - P->C_m.zero - (P->C_m.alpha * alpha) - P->C_m.q * (P->c * state.q / 2 * Va) ) / P->C_m.delta_e;

    /* Eq. 5.19 - Throttle Deflection - Uses linearizations of C_L(alpha) and C_D(alpha) */
    input.delta_t = sqrt( (2*P->m * (-state.r*state.v + state.q*state.w * P->g*sin(state.theta)) - P->row*pow(Va,2)*P->S*(C_X_alpha + C_X_q_alpha*(P->c*state.q)/(2*Va) + C_X_delta_e_alpha*input.delta_e))/(P->row*P->S_prop*P->C_prop*pow(P->k_motor,2)) + pow(Va,2)/pow(P->k_motor,2) );

    /* Eq. 5.20 - Aileron and Rudder Deflection */
    matrix_invert_2(&da_dr_mult1, &da_dr_mult1);         /* Invert the first matrix in place */
    matrix_multiply(&da_dr_mult1, &da_dr_mult2, &da_dr); /* Multiply the matrices */
    input.delta_a = da_dr.data[0];
    input.delta_r = da_dr.data[1];


    /* Now that we have all of the deflections, we can calculate f(x, u) using Eq. 5.3-5.12 */
    f_x_u.dh = state.u*cos(state.theta) - state.v*sin(phi)*cos(state.theta) - state.w*cos(phi)*cos(state.theta);

    f_x_u.du = state.r*state.v - state.q*state.w - P->g*sin(state.theta) + (P->row*pow(Va,2)*P->S)/(2*P->m) * (C_X_alpha + C_X_q_alpha*(P->c*state.q)/(2*Va) + C_X_delta_e_alpha*input.delta_e) + (P->row*P->S_prop*P->C_prop)/(2*P->m) * (pow(P->k_motor*input.delta_t,2) + pow(Va,2));

    f_x_u.dv = state.p*state.w - state.r*state.u - P->g*sin(state.theta) + (P->row*pow(Va,2)*P->S)/(2*P->m) * (P->C_Y.zero + P->C_Y.beta*beta + P->C_Y.p*(P->b*state.p)/(2*Va) + P->C_Y.r*(P->b*state.r)/(2*Va) + P->C_Y.delta_a*input.delta_a + P->C_Y.delta_r*input.delta_r);

    f_x_u.dw = state.q*state.u - state.p*state.v + P->g*cos(state.theta)*cos(phi) + (P->row*pow(Va,2)*P->S)/(2*P->m) * (C_Z_alpha + C_Z_q_alpha*(P->c*state.q)/(2*Va) + C_Z_delta_e_alpha*input.delta_e);

    f_x_u.dphi = state.p + state.q*sin(phi)*tan(state.theta) + state.r*cos(phi)*tan(state.theta);

    f_x_u.dtheta = state.q*cos(phi) - state.r*sin(phi);

    f_x_u.dpsi = state.q*sin(phi)*1/cos(state.theta) + state.r*cos(phi)*1/cos(state.theta);

    f_x_u.dp = P->Gamma.one*state.p*state.q - P->Gamma.two*state.q*state.r + 0.5*P->row*pow(Va,2)*P->S*P->b*(P->C_p.zero + P->C_p.beta*beta + P->C_p.p*(P->b*state.p)/(2*Va) + P->C_p.r*(P->b*state.r)/(2*Va) + P->C_p.delta_a*input.delta_a + P->C_p.delta_r*input.delta_r);

    f_x_u.dq = P->Gamma.five*state.p*state.r - P->Gamma.six*(pow(state.p,2) - pow(state.r,2)) + (P->row*pow(Va,2)*P->S*P->c)/(2*P->J.y) * (P->C_m.zero + P->C_m.alpha*alpha + P->C_m.q*(P->c*state.q)/(2*Va) + P->C_m.delta_e*input.delta_e);

    f_x_u.dr = P->Gamma.seven*state.p*state.q - P->Gamma.one*state.q*state.r + 0.5*P->row*pow(Va,2)*P->S*P->b*(P->C_r.zero + P->C_r.beta*beta + P->C_r.p*(P->b*state.p)/(2*Va) + P->C_r.r*(P->b*state.r)/(2*Va) + P->C_r.delta_a*input.delta_a + P->C_r.delta_r*input.delta_r);

    /* Return value is ||x - f(x,u)||^2, so the square of the magnitude of the difference between x and f(x,u).
     * Since norm is sqrt(x_1^2 + ... + x_n^2), the sqrt goes away when we square.*/
    return pow(xstar.dh-f_x_u.dh,2) + pow(xstar.du-f_x_u.du,2) + pow(xstar.dv-f_x_u.dv,2) + pow(xstar.dw-f_x_u.dw,2) + pow(xstar.dphi-f_x_u.dphi,2) + pow(xstar.dtheta-f_x_u.dtheta,2) + pow(xstar.dpsi-f_x_u.dpsi,2) + pow(xstar.dp-f_x_u.dp,2) + pow(xstar.dq-f_x_u.dq,2) + pow(xstar.dr-f_x_u.dr,2);
}

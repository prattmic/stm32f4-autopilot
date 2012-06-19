#include "pid.h"

float limit(float in, float l, float u) {
    if(in > u)
        return u;
    else if(in < l)
        return l;
    else
        return in;
}

float eval_pid(struct pid* this_pid, float set, float current, float Ts) {
    float error = set - current;
    float result_uncorrected;
    float result;

    /* Differentiator - Equation 6.30 */
    this_pid->d = (2*this_pid->tau - Ts)/(2*this_pid->tau + Ts)*this_pid->d + 2/(2*this_pid->tau + Ts)*(error-this_pid->past_error);
    
    /* Integrator - Equation 6.29 */
    this_pid->i += error + (Ts/2)*(error + this_pid->past_error);

    this_pid->past_error = error;

    result_uncorrected = this_pid->kp*error + this_pid->kd*this_pid->d + this_pid->ki*this_pid->i;

    result = limit(result_uncorrected, this_pid->windup_lowerlimit, this_pid->windup_upperlimit);

    /* Anti-windup scheme suggested by autopilot.m from http://uavbook.byu.edu/doku.php?id=project#chapter_6_-_autopilot_design */
    if (this_pid->ki != 0) {
        this_pid->i += Ts/this_pid->ki * (result - result_uncorrected);
    }

    return result;
}

/* Lateral Autopilot */

void lateral_autopilot(struct plane *P, float Va) {
    float a_phi1 = -0.5 * P->rho * Va*Va * P->S * P->b * P->C_p.p * P->b/(2*Va);
    float a_phi2 = 0.5 * P->rho * Va*Va * P->S * P->b * P->C_p.delta_a;
}

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

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



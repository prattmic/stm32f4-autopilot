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

float eval_pid(struct pid* this_pid, float set, float current, float Ts);
float limit(float in, float l, float u);

#endif /* PID_H_INCLUDED */

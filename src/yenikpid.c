//These will probably all be volatile
struct pid{
    float kp;                   //-----------------------------------------------------------------------------------
    float kd;                   //PID constants. let the tuning... BEGIN! These will need to be loaded from NV memory
    float ki;                   //-----------------------------------------------------------------------------------
    float i;                    //integrator
    float i_windup_upperlimit;
    float i_windup_lowerlimit;
    float output_limit
    float past_error;           //E[n-1]
    float past_de;              //D[n-1]
    float tau;                  //derivative time constant for this PID
};

float eval_pid(struct pid* this_pid, float set, float current, float Ts){
    float error = set - current;
    float de = (2*this_pid->tau - Ts)/(2*this_pid->tau + Ts)*this_pid->past_de + 2/(2*tau + Ts)*(error-this_pid->past_error);
    this_pid->i += error + (Ts/2)*(error + this_pid->past_error);
    this_pid->i = limit(this_pid->i, this_pid->i_windup_lowerlimit, this_pid->i_windup_upperlimit);
    this_pid->past_error = error;
    this_pid->past_de = de;
    return this_pid->kp*error + this_pid->kd*de + this_pid->ki*this_pid->i;
}

float limit(float in, float l, float u){
    if(in > u)
        return u;
    else if(in < l)
        return l;
    else
        return in;
}



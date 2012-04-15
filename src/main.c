/* Michael Pratt <michael@pratt.im>
 * Simplified code from https://github.com/jeremyherbert/stm32-templates/
 * Building the Autopilot described in Small Unmanned Aircraft by Beard
 * Rev. Chapter 6 */
#include "yenikpid.h"
typedef unsigned long uint32_t;

#define RCC_CR      0x40023800                              /* RCC Clock Control Register */
#define RCC_AHB1ENR *(volatile uint32_t *) (RCC_CR + 0x30)  /* AHB1 Enable Register */

#define GPIOD       0x40020c00                              /* Port D base address */
#define GPIOD_MODER *(volatile uint32_t *) (GPIOD + 0x00)   /* Port D mode register */
#define LED_ODR     *(volatile uint32_t *) (GPIOD + 0x14)   /* LED Output Data Register */

/* Structures */
struct pidsettings {
    float kp;
    float ki;
    float kd;
    float Ts;
    float tau;
    float upper_limit;
    float lower_limit;
    volatile float *integrator;
    volatile float *differentiator;
    volatile float *error_d1;
};

/*globals*/
volatile struct pid testpid;


void Delay(volatile uint32_t nCount);

/* Generic Function */
float pid_loop(volatile float *integrator, volatile float *differentiator, volatile float *error_d1, float y_c, float y, float kp, float ki, float kd, float Ts, float tau, float upper_limit, float lower_limit);
void reset_pid(struct pidsettings *settings);
float sat(float in, float upper_limit, float lower_limit);
float eval_pid(struct pid* this_pid, float set, float current, float Ts);

/* Autopilot Functions */
float airspeed_with_pitch_hold(float Va_c, float Va, struct pidsettings *settings, uint32_t reset);

int main(void) {
    //-----------------------TODO: INITIALZE PIDS FROM NV MEMORY---------------------------------------//
    /* Enable Port D Clock
    * See docs/stm32f4_ref.pdf page 110 for description of RCC_AHB1ENR */
    RCC_AHB1ENR |= (1 << 3);

    /* Set pins to output
    * See docs/stm32f4_ref.pdf page 148 for description of GPIOD_MODER */
    GPIOD_MODER |= (1 << (12 * 2)) | (1 << (13 * 2)) | (1 << (14 * 2)) | (1 << (15 * 2));

    while (1) {
        /* Toggle LEDs */
        LED_ODR ^= (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);

        Delay(100000000);
    }
}

void Delay(volatile uint32_t nCount) {
    /* Simulate wind-up */
    volatile float result = 0;
    float integr, diff, old_err;
    struct pidsettings testpidsettings;
    testpidsettings.kp = 3.14;
    testpidsettings.ki = 3.14;
    testpidsettings.kd = 3.14;
    testpidsettings.Ts = 3.14;
    testpidsettings.tau = 3.14;
    testpidsettings.upper_limit = nCount;
    testpidsettings.lower_limit = -(signed long)nCount;
    testpidsettings.integrator = &integr;
    testpidsettings.differentiator = &diff;
    testpidsettings.error_d1 = &old_err;

    reset_pid(&testpidsettings);

    while(result != nCount && result != -(signed long)nCount) {
        result = pid_loop(testpidsettings.integrator, testpidsettings.differentiator, testpidsettings.error_d1, 100, 0, testpidsettings.kp, testpidsettings.ki, testpidsettings.kd, testpidsettings.Ts, testpidsettings.tau, testpidsettings.upper_limit, testpidsettings.lower_limit);
    }
}

/* Generic PID function */
float pid_loop(volatile float *integrator, volatile float *differentiator, volatile float *error_d1, float y_c, float y, float kp, float ki, float kd, float Ts, float tau, float upper_limit, float lower_limit) {
    float error;
    float u;
    float u_sat;

    error = y_c - y;        /* Commanded - Actual */
    *integrator = *integrator + (Ts/2) * (error + *error_d1);      /* Equation 6.29 */
    *differentiator = (2*tau-Ts)/(2*tau+Ts) * *differentiator + 2/(2*tau+Ts) * (error - *error_d1);        /* Equation 6.30 */

    *error_d1 = error;

    u = kp * error + ki * *integrator + kd * *differentiator;       /* See pg. 116 */

    /* abs(u) <= limit must be true */
    u_sat = sat(u, upper_limit, lower_limit);

    /* Anti-windup scheme suggested by autopilot.m from http://uavbook.byu.edu/doku.php?id=project#chapter_6_-_autopilot_design */
    if (ki != 0) {
        *integrator = *integrator + Ts/ki * (u_sat - u);
    }

    return u_sat;
}

void reset_pid(struct pidsettings *settings) {
    *settings->integrator = 0;
    *settings->differentiator = 0;
    *settings->error_d1 = 0;
}

/* Saturation function.  Ensures that result is within set limits. */
float sat(float in, float upper_limit, float lower_limit) {
    if (in > upper_limit) {
        return upper_limit;
    }
    else if (in < lower_limit) {
        return lower_limit;
    }
    else {
        return in;
    }
}

float airspeed_with_pitch_hold(float Va_c, float Va, struct pidsettings *settings, uint32_t reset) {
    if (reset) {
        reset_pid(settings);
    }

    return pid_loop(settings->integrator, settings->differentiator, settings->error_d1, Va_c, Va, settings->kp, settings->ki, settings->kd, settings->Ts, settings->tau, settings->upper_limit, settings->lower_limit);
}

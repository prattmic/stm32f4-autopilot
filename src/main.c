/* Michael Pratt <michael@pratt.im>
 * Vastly simplified the code from https://github.com/jeremyherbert/stm32-templates/
 * Building the Autopilot described in Small Unmanned Aircraft by Beard
 * Rev. Chapter 6 */

#include "system.h"
//#include "pid.h"
#include "trim.h"

#define RCC_AHB1ENR *(volatile uint32_t *) (RCC_BASE + 0x30)    /* AHB1 Enable Register */

#define GPIOD_BASE  (AHB1PERIPH_BASE + 0x0C00)                  /* GPIO Port D base address */
#define GPIOD_MODER *(volatile uint32_t *) (GPIOD_BASE + 0x00)  /* Port D mode register */
#define LED_ODR     *(volatile uint32_t *) (GPIOD_BASE + 0x14)  /* LED Output Data Register */

void Delay(volatile uint32_t nCount);

int main(void) {
    struct trim_out result;
    /* Using Aersonde UAV Parameters, pg. 288 */
    struct plane UAV = { 0 };
    float alpha = 0;
    float beta = 0;
    float phi = 0;
    float Va = 20;
    float R = 10;
    float gamma = 0.053;
    //-----------------------TODO: INITIALZE PIDS FROM NV MEMORY---------------------------------------//
    /* Enable Port D Clock
    * See docs/stm32f4_ref.pdf page 110 for description of RCC_AHB1ENR */
    RCC_AHB1ENR |= (1 << 3);

    /* Set pins to output
    * See docs/stm32f4_ref.pdf page 148 for description of GPIOD_MODER */
    GPIOD_MODER |= (1 << (12 * 2)) | (1 << (13 * 2)) | (1 << (14 * 2)) | (1 << (15 * 2));

    /* See trim.h for explaination of variables. */
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
    /* That was fun, wasn't it? */

    /* Now compute some extra values we will need */
    compute_params(&UAV);

    /* Let's get the value of J! */
    result = minimize_J(alpha,beta,phi,Va,R,gamma,&UAV);  /* 0deg alpha, beta, and phi, Va=20, R=10, gamma=5deg */

    while (1) {
        /* Toggle LEDs */
        LED_ODR ^= (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);

        Delay(100000000);
    }
}

void Delay(volatile uint32_t nCount) {
    while (--nCount) {
        float result = 3.14;
        result = result * nCount;
    }
}

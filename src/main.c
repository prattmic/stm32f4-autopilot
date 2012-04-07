/* Michael Pratt <michael@pratt.im>
 * Simplified code from https://github.com/jeremyherbert/stm32-templates/
 * Building the Autopilot described in Small Unmanned Aircraft by Beard
 * Rev. Chapter 6 */

typedef unsigned long uint32_t;

#define RCC_CR      0x40023800                              /* RCC Clock Control Register */
#define RCC_AHB1ENR *(volatile uint32_t *) (RCC_CR + 0x30)  /* AHB1 Enable Register */

#define GPIOD       0x40020c00                              /* Port D base address */
#define GPIOD_MODER *(volatile uint32_t *) (GPIOD + 0x00)   /* Port D mode register */
#define LED_ODR     *(volatile uint32_t *) (GPIOD + 0x14)   /* LED Output Data Register */

void Delay(volatile uint32_t nCount);

float pidloop(float y_c, float y, uint32_t flag, float kp, float ki, float kd, float limit, float Ts, float tau);

int main(void) {

    /* Enable Port D Clock
    * See docs/stm32f4_ref.pdf page 110 for description of RCC_AHB1ENR */
    RCC_AHB1ENR |= (1 << 3);

    /* Set pins to output
    * See docs/stm32f4_ref.pdf page 148 for description of GPIOD_MODER */
    GPIOD_MODER |= (1 << (12 * 2)) | (1 << (13 * 2)) | (1 << (14 * 2)) | (1 << (15 * 2));

    while (1) {
        /* Toggle LEDs */
        LED_ODR ^= (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);

        Delay(10000000);
    }
}

void Delay(volatile uint32_t nCount) {
    /* Simulate wind-up */
    float limit = nCount;
    volatile float result = pidloop(100, 0, 1, 1, 1, 1, limit, 1, 1);     /* Reset values */

    while(result != limit && result != -limit) {
        result = pidloop(100, 0, 0, 1, 1, 1, limit, 1, 1);
    }
}

/* Vars for pidloop */
volatile float integrator;
volatile float differentiator;
volatile float error_d1;        /* Previous error */

float pidloop(float y_c, float y, uint32_t flag, float kp, float ki, float kd, float limit, float Ts, float tau) {
    float error;
    float u;

    if (flag) {     /* Reset values */
        integrator = 0;
        differentiator = 0;
        error_d1 = 0;
    }

    error = y_c - y;        /* Commanded - Actual */
    integrator = integrator + (Ts/2) * (error + error_d1);      /* Equation 6.29 */
    differentiator = (2*tau-Ts)/(2*tau+Ts) * differentiator + 2/(2*tau+Ts) * (error - error_d1);        /* Equation 6.30 */

    error_d1 = error;

    u = kp*error + ki*integrator + kd*differentiator;       /* See pg. 116 */

    /* abs(u) <= limit must be true */
    if (u > limit) {
        return limit;
    }
    else if (u < -limit) {
        return -limit;
    }
    else {
        return u;
    }
}


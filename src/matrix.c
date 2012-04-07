/* Michael Pratt <michael@pratt.im>
 * Do matrix operations */

typedef unsigned long uint32_t;

#define RCC_CR      0x40023800                              /* RCC Clock Control Register */
#define RCC_AHB1ENR *(volatile uint32_t *) (RCC_CR + 0x30)  /* AHB1 Enable Register */

#define GPIOD       0x40020c00                              /* Port D base address */
#define GPIOD_MODER *(volatile uint32_t *) (GPIOD + 0x00)   /* Port D mode register */
#define LED_ODR     *(volatile uint32_t *) (GPIOD + 0x14)   /* LED Output Data Register */

void Delay(volatile uint32_t nCount);
void matrix_multiply(volatile float *one, volatile float *two, volatile float *out);

volatile float one[] = {1, 2, 3,
                        4, 5, 6,
                        7, 8, 9};

volatile float two[] = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};

volatile float out[] = {0, 0, 0,
                        0, 0, 0,
                        0, 0, 0};

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

        Delay(1000000);
    }
}

void Delay(volatile uint32_t nCount) {
  while(nCount--)
  {
      matrix_multiply(one, two, out);
  }
}

void matrix_multiply(volatile float *one, volatile float *two, volatile float *out) {
    /* Dumb matrix multiplication expects 3x3 matrix */

    out[0] = one[0]*two[0] + one[1]*two[3] + one[2]*two[6];
    out[1] = one[0]*two[1] + one[1]*two[4] + one[2]*two[7];
    out[2] = one[0]*two[2] + one[1]*two[5] + one[2]*two[8];
    out[3] = one[3]*two[0] + one[4]*two[3] + one[5]*two[6];
    out[4] = one[3]*two[1] + one[4]*two[4] + one[5]*two[7];
    out[5] = one[3]*two[2] + one[3]*two[5] + one[5]*two[8];
    out[6] = one[6]*two[0] + one[7]*two[3] + one[8]*two[6];
    out[7] = one[6]*two[1] + one[7]*two[4] + one[8]*two[7];
    out[8] = one[6]*two[2] + one[7]*two[5] + one[8]*two[8];
}

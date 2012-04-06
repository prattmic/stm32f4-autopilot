/* Michael Pratt <michael@pratt.im>
 * Vastly simplified the code from https://github.com/jeremyherbert/stm32-templates/ */

typedef unsigned long uint32_t;

#define RCC_CR      0x40023800                              /* RCC Clock Control Register */
#define RCC_AHB1ENR *(volatile uint32_t *) (RCC_CR + 0x30)  /* AHB1 Enable Register */

#define GPIOD       0x40020c00                              /* Port D base address */
#define GPIOD_MODER *(volatile uint32_t *) (GPIOD + 0x00)   /* Port D mode register */
#define LED_ODR     *(volatile uint32_t *) (GPIOD + 0x14)   /* LED Output Data Register */

void Delay(volatile uint32_t nCount);

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

        Delay(5000000);
    }
}

void Delay(volatile uint32_t nCount) {
  float one;
  while(nCount--)
  {
      one = (float) nCount*3.141592f;
  }
}

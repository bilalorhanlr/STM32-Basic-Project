#include "stm32f4xx.h"

void GPIO_Init(void);
void Timer2_Init(void);
void ADC1_Init(void);
uint32_t Read_ADC(void);
void delay(uint32_t time);

int main(void) {
    GPIO_Init();
    Timer2_Init();
    ADC1_Init();

    uint32_t adc_value;
    uint32_t dead_zone = 200; // Ölü bölge değeri (ADC değeri 0-4095)

    while (1) {
        adc_value = Read_ADC(); // Potansiyometreden ADC okuma

        // Ölü bölge kontrolü
        if (adc_value < (2048 - dead_zone)) {
            // Potansiyometre sola döndü (ters yön)
            GPIOA->ODR |= GPIO_ODR_OD2;  // PA2 HIGH
            GPIOA->ODR &= ~GPIO_ODR_OD3; // PA3 LOW (GND)
            uint32_t pwm_value = ((2048 - adc_value) * (TIM2->ARR + 1)) / 2048; // PWM hesapla
            TIM2->CCR4 = pwm_value;
        } else if (adc_value > (2048 + dead_zone)) {
            // Potansiyometre sağa döndü (ileri yön)
            GPIOA->ODR &= ~GPIO_ODR_OD2; // PA2 LOW (GND)
            GPIOA->ODR |= GPIO_ODR_OD3;  // PA3 HIGH
            uint32_t pwm_value = ((adc_value - 2048) * (TIM2->ARR + 1)) / 2048; // PWM hesapla
            TIM2->CCR4 = pwm_value;
        } else {
            // Ölü bölgede, motor durdur
            TIM2->CCR4 = 0;
        }

        delay(1000); // Gecikme
    }
}

void GPIO_Init(void) {
    // Enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA2 and PA3 as output
    GPIOA->MODER |= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0; // General purpose output mode for PA2 and PA3

    // Set PA3 as alternate function for PWM output
    GPIOA->MODER &= ~GPIO_MODER_MODER3; // Reset mode for PA3
    GPIOA->MODER |= GPIO_MODER_MODER3_1; // Alternate function mode for PA3
    GPIOA->AFR[0] |= (1 << 12); // AF1 for PA3

    // Set PA5 as analog input
    GPIOA->MODER |= GPIO_MODER_MODER5; // Analog mode for PA5
}

void Timer2_Init(void) {
    // Enable clock for TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set prescaler value
    TIM2->PSC = 160 - 1; // Adjust prescaler to get a higher PWM frequency (10 kHz)

    // Set auto-reload value
    TIM2->ARR = 1000 - 1; // Set auto-reload value for 10 kHz PWM frequency

    // Set compare value (initially 0)
    TIM2->CCR4 = 0;

    // Set PWM mode 1 for channel 4
    TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    // Enable output compare for channel 4
    TIM2->CCER |= TIM_CCER_CC4E;

    // Enable counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

void ADC1_Init(void) {
    // Enable clock for ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // ADC1 regular channel 5 (PA5) configuration
    ADC1->SQR3 |= 5; // First conversion in regular sequence is channel 5

    // Enable ADC1
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint32_t Read_ADC(void) {
    // Start ADC conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Read ADC value
    return ADC1->DR;
}

void delay(uint32_t time) {
    while (time--) {
        __NOP();
    }
}

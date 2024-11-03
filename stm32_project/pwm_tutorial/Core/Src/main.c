#include "stm32f4xx.h"

// Fonksiyon Prototipleri
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM3_Init(void);
void delay(uint32_t time);

int main(void) {
    SystemClock_Config();
    GPIO_Init();
    TIM3_Init();

    while (1) {
        // PWM sinyalinin duty cycle'ını artır ve azalt
        for (int duty = 0; duty <= 1000; duty++) {
            TIM3->CCR1 = duty; // Duty cycle'ı ayarla
            delay(2000); // Küçük bir gecikme
        }
        for (int duty = 1000; duty >= 0; duty--) {
            TIM3->CCR1 = duty; // Duty cycle'ı ayarla
            delay(2000); // Küçük bir gecikme
        }
    }
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION; // HSI osilatörünü etkinleştir
    while (!(RCC->CR & RCC_CR_HSIRDY)); // HSI hazır olana kadar bekle
    RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI'yı sistem saati olarak seç
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // HSI'nın sistem saati olduğunu doğrula
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA saatini etkinleştir

    // PA6 pinini alternatif fonksiyon olarak ayarla (TIM3 CH1)
    GPIOA->MODER &= ~(0x3 << (6 * 2));
    GPIOA->MODER |= (0x2 << (6 * 2));
    GPIOA->AFR[0] |= (0x2 << (6 * 4)); // PA6 pinini TIM3_CH1 fonksiyonuna ayarla (AF2)
}

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // TIM3 saatini etkinleştir

    TIM3->PSC = 16 - 1; // Prescaler değeri (16MHz / 16 = 1MHz)
    TIM3->ARR = 1000 - 1; // Auto-reload register (1kHz PWM frekansı)

    TIM3->CCMR1 |= (0x6 << 4); // PWM mod 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Çıkış ön yüklemesini etkinleştir

    TIM3->CCER |= TIM_CCER_CC1E; // Çıkışı etkinleştir

    TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload ön yüklemesini etkinleştir
    TIM3->CR1 |= TIM_CR1_CEN; // Timer'ı etkinleştir

    // Başlangıç duty cycle (initial duty cycle 0%)
    TIM3->CCR1 = 0; // CCR1 registerını 0 yap (duty cycle %0)
}

void delay(uint32_t time) {
    while (time--) {
        __NOP(); // No Operation (boş işlem)
    }
}

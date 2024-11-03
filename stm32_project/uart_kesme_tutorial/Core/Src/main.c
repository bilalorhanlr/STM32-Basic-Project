#include "stm32f4xx.h"

// Prototipler
void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_Init(void);
void NVIC_Config(void);
void LED_On(uint16_t pin);
void LED_Off(uint16_t pin);

// Değişkenler
volatile char received_char;

int main(void) {

}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION; // HSI osilatörünü etkinleştir
    while (!(RCC->CR & RCC_CR_HSIRDY)); // HSI hazır olana kadar bekle
    RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI'yı sistem saati olarak seç
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // HSI'nın sistem saati olduğunu doğrula
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB önbölücü: 1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // APB1 önbölücü: 1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 önbölücü: 1
}

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // GPIOD saatini etkinleştir
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA saatini etkinleştir

    // PD12, PD13, PD14, PD15 pinlerini çıkış olarak ayarla
    GPIOD->MODER &= ~((0x3 << (12 * 2)) | (0x3 << (13 * 2)) | (0x3 << (14 * 2)) | (0x3 << (15 * 2)));
    GPIOD->MODER |= (0x1 << (12 * 2)) | (0x1 << (13 * 2)) | (0x1 << (14 * 2)) | (0x1 << (15 * 2));

    // PA2 ve PA3 pinlerini alternatif fonksiyon olarak ayarla
    GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    GPIOA->MODER |= (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] |= (0x7 << (2 * 4)) | (0x7 << (3 * 4)); // PA2 ve PA3 için AF7 (USART2) alternatif fonksiyonunu ayarla
}

void USART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // USART2 saatini etkinleştir

    USART2->BRR = 0x0683; // Baudrate 9600 (16 MHz sistem saatinde)
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; // Receive, Transmit ve Receive interrupt enable
    USART2->CR1 |= USART_CR1_UE; // USART2 etkinleştir
}

void NVIC_Config(void) {
    NVIC_SetPriority(USART2_IRQn, 1); // USART2 kesmesi için öncelik ayarla
    NVIC_EnableIRQ(USART2_IRQn); // USART2 kesmesini etkinleştir
}

void USART2_IRQHandler(void) {
    if (USART2->SR & (1 << 5)) { // RXNE (Receive not empty) bayrağını kontrol et
        received_char = USART2->DR; // Alınan karakteri değişkene ata
        USART2->DR = received_char; // EKRANA GERİ YAZDIRMA
        while(!(USART2->SR & (1 << 6))); //TC BİTİ KONTROLÜ
    }
}

void LED_On(uint16_t pin) {
    GPIOD->BSRR = (1 << pin); // İlgili pini HIGH yap
}

void LED_Off(uint16_t pin) {
    GPIOD->BSRR = (1 << (pin + 16)); // İlgili pini LOW yap
}

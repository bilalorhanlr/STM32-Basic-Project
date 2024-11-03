#include "stm32f4xx.h"
#include <stdio.h>

// Fonksiyon Prototipleri
void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_Init(void);
void TIM3_Init(void);
void HCSR04_Read(void);
void delay_us(uint32_t us);
void USART_Write(char ch);
void USART_WriteString(char* str);
void format_and_send_distance(float distance);

// Mesafe ölçüm değişkeni
volatile float distance;

int main(void) {

    SystemClock_Config();
    GPIO_Init();
    USART2_Init();
    TIM3_Init();

    while (1) {

    }
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

    // PA2 ve PA3 pinlerini alternatif fonksiyon olarak ayarla (USART2 TX/RX)
    GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    GPIOA->MODER |= (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->AFR[0] |= (0x7 << (2 * 4)) | (0x7 << (3 * 4));

    // PD12 (Trig) ve PD13 (Echo) pinlerini çıkış ve giriş olarak ayarla
    GPIOD->MODER &= ~((0x3 << (12 * 2)) | (0x3 << (13 * 2)));
    GPIOD->MODER |= (0x1 << (12 * 2)); // PD12 çıkış (Trig)
    GPIOD->MODER |= (0x0 << (13 * 2)); // PD13 giriş (Echo)
}

void USART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // USART2 saatini etkinleştir

    USART2->BRR = 0x0683; // Baudrate 9600 (16 MHz sistem saatinde)
    USART2->CR1 = USART_CR1_RE | USART_CR1_TE; // Receive ve Transmit enable
    USART2->CR1 |= USART_CR1_UE; // USART2 etkinleştir
}

void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // TIM3 saatini etkinleştir

    TIM3->PSC = 64 - 1; // Prescaler değeri (16MHz / 16 = 1MHz)
    TIM3->ARR = 0xFFFF; // Auto-reload register (maksimum değer)
    TIM3->DIER |= TIM_DIER_UIE; // Update kesmesini etkinleştir
    TIM3->CR1 |= TIM_CR1_CEN; // Timer'ı etkinleştir

    NVIC_EnableIRQ(TIM3_IRQn); // Timer kesmesini NVIC'te etkinleştir
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; // Kesme bayrağını temizle

        HCSR04_Read();
        format_and_send_distance(distance);
    }
}

void delay_us(uint32_t us) {
    TIM3->CNT = 0; // Sayaç registerını sıfırla
    while (TIM3->CNT < us); // Belirtilen mikrosaniye kadar bekle
}

void HCSR04_Read(void) {
    // Trig pini düşük yap
    GPIOD->ODR &= ~(1 << 12);
    delay_us(2);

    // Trig pini yüksek yap (10us süreyle)
    GPIOD->ODR |= (1 << 12);
    delay_us(10);
    GPIOD->ODR &= ~(1 << 12);

    // Echo pini yüksek olana kadar bekle (maksimum bekleme süresi 1 saniye)
    uint32_t timeout = 1000000;
    while (!(GPIOD->IDR & (1 << 13)) && timeout > 0) {
        timeout--;
    }

    if (timeout == 0) {
        distance = 0; // Eğer timeout olduysa, mesafe 0 olarak ayarlanır
        return;
    }

    // Yükselen kenar tespit edildiğinde zamanlayıcıyı başlat
    TIM3->CNT = 0;

    // Echo pini düşük olana kadar bekle (maksimum bekleme süresi 1 saniye)
    timeout = 1000000;
    while ((GPIOD->IDR & (1 << 13)) && timeout > 0) {
        timeout--;
    }

    if (timeout == 0) {
        distance = 0; // Eğer timeout olduysa, mesafe 0 olarak ayarlanır
        return;
    }

    // Zamanlayıcıyı durdur ve süreyi a
    uint32_t timeElapsed = TIM3->CNT;

    // Mesafeyi hesapla (cm cinsinden)
    distance = (timeElapsed * 0.0343) / 2.0;
}

void USART_Write(char ch) {
    while (!(USART2->SR & USART_SR_TXE)); // Gönderim tamponu boş olana kadar bekle
    USART2->DR = ch;
}

void USART_WriteString(char* str) {
    while (*str) {
        USART_Write(*str++);
    }
}

void format_and_send_distance(float distance) {
    char buffer[50];
    sprintf(buffer, "Distance: %.1f cm\r\n", (4*distance));
    USART_WriteString(buffer);
}

#include "stm32f4xx.h"

void System_Clock_Init(void)
{
    RCC->CR |= (1 << 0);                // Sistem saatini HSI seç
    while (!(RCC->CR & RCC_CR_HSIRDY)); // HSI hazır olana kadar bekle
    RCC->CFGR &= ~(3 << 0);             // HSI'yi sistem saati olarak yapılandır
    while ((RCC->CFGR & (3 << 2)) != (0 << 2));
    RCC->AHB1ENR |= (1 << 0);           // GPIOA saatini etkinleştir
    RCC->AHB1ENR |= (1 << 1);           // GPIOB saatini etkinleştir
    RCC->AHB1ENR |= (1 << 3);           // GPIOD saatini etkinleştir (LED için)
    RCC->APB2ENR |= (1 << 12);          // SPI1 için saat etkinleştir
}

void GPIO_Init(void)
{
    // PA4, PA5, PA6, PA7 pinlerini alternatif fonksiyon (AF) moduna ayarla
    GPIOA->MODER &= ~((0x3 << (2 * 5)) | (0x3 << (2 * 6)) | (0x3 << (2 * 7)));  // PA4, PA5, PA6, PA7 pinlerini temizle
    GPIOA->MODER |= ((0x2 << (2 * 5)) | (0x2 << (2 * 6)) | (0x2 << (2 * 7)));   // PA4, PA5, PA6, PA7 pinlerini AF moduna ayarla
    GPIOA->AFR[0] |= (5 << (4 * 5)) | (5 << (4 * 6)) | (5 << (4 * 7));            // PA4, PA5, PA6, PA7 için AF5 seç

    // PA1, PA3, PA8, PA10 pinlerini çıkış (output) moduna ayarla
    GPIOA->MODER &= ~((0x3 << (2 * 1)) | (0x3 << (2 * 3)) | (0x3 << (2 * 8)) | (0x3 << (2 * 10))); // PA1, PA3, PA8, PA10 pinlerini temizle
    GPIOA->MODER |= ((0x1 << (2 * 1)) | (0x1 << (2 * 3)) | (0x1 << (2 * 8)) | (0x1 << (2 * 10)));  // PA1, PA3, PA8, PA10 pinlerini output moduna ayarla

    // PB2, PB4, PB5, PB6, PB9, PB11, PB12 pinlerini çıkış (output) moduna ayarla
    GPIOB->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (2 * 4)) | (0x3 << (2 * 5)) | (0x3 << (2 * 6)) |
                      (0x3 << (2 * 9)) | (0x3 << (2 * 11)) | (0x3 << (2 * 12)));                    // PB2, PB4, PB5, PB6, PB9, PB11, PB12 pinlerini temizle
    GPIOB->MODER |= ((0x1 << (2 * 2)) | (0x1 << (2 * 4)) | (0x1 << (2 * 5)) | (0x1 << (2 * 6)) |
                     (0x1 << (2 * 9)) | (0x1 << (2 * 11)) | (0x1 << (2 * 12)));                    // PB2, PB4, PB5, PB6, PB9, PB11, PB12 pinlerini output moduna ayarla

//    // PD15 pinini çıkış (output) moduna ayarla (Mavi LED)
//    GPIOD->MODER &= ~(0x3 << (2 * 14)); // PD14 pinini temizle
//    GPIOD->MODER |= (0x1 << (2 * 14));  // PD14 pinini output moduna ayarla
//
//    // PD15 pinini çıkış (output) moduna ayarla (Mavi LED)
//    GPIOD->MODER &= ~(0x3 << (2 * 15)); // PD15 pinini temizle
//    GPIOD->MODER |= (0x1 << (2 * 15));  // PD15 pinini output moduna ayarla


}

void SPI1_Init(void)
{
    SPI1->CR1 &= ~(1 << 0);      // CPHA (Saat Fazı)
    SPI1->CR1 &= ~(1 << 1);      // CPOL (Saat Kutbu)
    SPI1->CR1 &= ~(1 << 2);      // MSTR (Master/Slave Seçimi)
    SPI1->CR1 &= ~(0x7 << 3);    // BR (Baud Rate)
    SPI1->CR1 |= (0x3 << 3);     // SPI hızını ayarla
    SPI1->CR1 &= ~(1 << 11);     // DFF (Data Frame Format)
    SPI1->CR1 |= (SPI_CR1_SPE);  // SPI'yi etkinleştir

    // SS pinini (PA4) çıkış moduna ayarla
        GPIOA->MODER &= ~(0x3 << (2 * 4)); // PA4 pinini temizle
        GPIOA->MODER |=  (0x1 << (2 * 4)); // PA4'ü output moduna ayarla
        GPIOA->BSRR = (1 << (4 + 16)); // SS pinini HIGH yap (slave'i devre dışı bırak)
}

void UART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // UART2 saatini etkinleştir

    GPIOA->MODER |= (2 << 4) | (2 << 6);  // PA2 ve PA3'ü alternatif fonksiyona ayarla
    GPIOA->AFR[0] |= (7 << 8) | (7 << 12);  // PA2 ve PA3 için AF7 (UART2) ayarla

    USART2->BRR = 0x0683;  // Baud rate 9600 bps
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // UART'ı etkinleştir, TX ve RX'i etkinleştir
}

void SPI_Transmit(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));  // TXE bayrağının set olmasını bekle
    SPI1->DR = data;  // Veriyi gönder
}

uint8_t Spi_Recieve(void)
{
    if (SPI1->SR & SPI_SR_RXNE)
        return (uint8_t)(SPI1->DR);
    return 0;
}

void LED_On(uint16_t pin) {
    GPIOD->BSRR = (1 << pin); // İlgili pini HIGH yap
}

void UART2_Write(uint8_t data) {
    while (!(USART2->SR & USART_SR_TXE));  // TXE bayrağının set olmasını bekle
    USART2->DR = (0x30 + data); // Veriyi gönder
    LED_On(14);
}

void LED_Off(uint16_t pin) {
    GPIOD->BSRR = (1 << (pin + 16)); // İlgili pini LOW yap
}

void delay_ms(int delay) {
    for (int i = 0; i < delay * 1000; i++);
}

int main(void)
{
    System_Clock_Init();
    GPIO_Init();
    SPI1_Init();
    UART2_Init();

    uint8_t number = 1;
    while (1) {
        GPIOA->BSRR = (1 << (4 + 16));  // SS pinini LOW yap (slave'i etkinleştir)
        SPI_Transmit(number++);  // Veriyi gönder
        GPIOA->BSRR = (1 << 4);  // SS pinini HIGH yap (slave'i devre dışı bırak)
        delay_ms(1000); // 1 saniye bekle
    }
}
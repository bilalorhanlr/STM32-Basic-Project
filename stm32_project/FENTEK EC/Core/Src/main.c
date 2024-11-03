#include "stm32f1xx_it.h"

void SPI1_Init(void) {
    // GPIOA portu ve SPI1 için saat sinyallerini etkinleştir
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

    // PA5 (SCK), PA6 (MISO), PA7 (MOSI) pinlerini ayarla
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
    GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_CNF7_1); // PA5 ve PA7 Input mode floating
    GPIOA->CRL |= (GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_1); // PA6 Output mode, AF push-pull

    // PA4 (NSS) pinini ayarla
    GPIOA->CRL &= ~GPIO_CRL_CNF4;
    GPIOA->CRL |= GPIO_CRL_CNF4_1; // PA4 Input mode floating

    // SPI1 yapılandırması
    SPI1->CR1 = 0; // SPI1 kontrol register'ını sıfırla
    SPI1->CR1 |= SPI_CR1_MSTR; // Master değil, Slave modundayız
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Yazılım NSS yönetimi
    SPI1->CR1 |= SPI_CR1_SPE; // SPI'yi etkinleştir
}

uint8_t SPI1_ReceiveData(void) {
    // SPI'den veri gelmesini bekle
    while (!(SPI1->SR & SPI_SR_RXNE));
    // Gelen veriyi oku
    return SPI1->DR;
}

int main(void) {
    uint8_t received_data;

    SPI1_Init();  // SPI1'i başlat

    while (1) {
        received_data = SPI1_ReceiveData();
        // Gelen veriyi işleyebilirsin
    }
}

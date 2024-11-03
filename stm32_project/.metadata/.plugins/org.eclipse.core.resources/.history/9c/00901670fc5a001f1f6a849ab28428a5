#include "stm32f4xx.h"

// Sensör verileri için struct
typedef struct {
    uint8_t sensor_id;    // Sensör ID'si (1, 2, 3 gibi)
    float temperature;    // Sıcaklık değeri (bu örnekte kullanmıyoruz ama struct'a dahil edebiliriz)
    uint32_t timestamp;   // Zaman damgası
} SensorData;

// STM32 ve Mega arasındaki mesajlar için struct
typedef struct {
    SensorData sensors[3]; // 3 farklı sensörden veri
} DataPacket;

void SPI1_Init(void) {
    // GPIO ayarları (PA12, PA13, PA14 output modunda)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock'u etkinleştir
    GPIOA->MODER |= (1 << (2 * 12)) | (1 << (2 * 13)) | (1 << (2 * 14)); // PA12, PA13, PA14 çıkış modunda

    // SPI1 ayarları (Slave modunda)
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   // SPI1 clock'u etkinleştir
    SPI1->CR1 = 0; // CR1 register'ını resetle
    SPI1->CR1 &= ~SPI_CR1_BR;  // Baud rate'yi en düşük değere ayarla
    SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA; // Clock polarity ve phase
    SPI1->CR1 &= ~SPI_CR1_MSTR; // Slave modu
    SPI1->CR1 |= SPI_CR1_SPE; // SPI'yi etkinleştir
}

void control_pins(DataPacket* packet) {
    for (int i = 0; i < 3; i++) {
        switch (packet->sensors[i].sensor_id) {
            case 1:
                GPIOA->ODR |= (1 << 12); // PA12 pinini HIGH yap
                break;
            case 2:
                GPIOA->ODR |= (1 << 13); // PA13 pinini HIGH yap
                break;
            case 3:
                GPIOA->ODR |= (1 << 14); // PA14 pinini HIGH yap
                break;
        }
    }
}

int main(void) {
    SPI1_Init();  // SPI1'i başlat
    DataPacket receivedPacket;

    while (1) {
        // STM32, SPI'den veri alacak
        uint8_t* buffer = (uint8_t*)&receivedPacket;
        for (int i = 0; i < sizeof(DataPacket); i++) {
            while (!(SPI1->SR & SPI_SR_RXNE)); // RX buffer'ı bekle
            buffer[i] = SPI1->DR; // Gelen veriyi al
        }

        // Pinleri kontrol et
        control_pins(&receivedPacket);
    }
}

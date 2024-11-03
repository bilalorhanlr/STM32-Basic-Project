#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Struct tanımı
typedef struct {
    uint8_t id;
    int16_t sensor_value;
    float temperature;
} SensorData;

#define I2C_ADDRESS 0x52  // STM32'nin I2C adresi

// Zamanlayıcı fonksiyonu (basit bir gecikme)
void delay(volatile uint32_t s) {
    for(s; s>0; s--);
}

void System_Clock_Init(void)
{
    RCC->CR |= RCC_CR_HSION;  // HSI saat kaynağını etkinleştir
    while (!(RCC->CR & RCC_CR_HSIRDY));  // HSI hazır olana kadar bekle

    // HSI'yi sistem saati olarak ayarla
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // HSI sistem saati olarak seçilene kadar bekle

    // GPIOB, GPIOD ve I2C1 saatini etkinleştir
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // USART2 saatini etkinleştir
}

void GPIO_Init(void)
{
    // I2C Pinleri (PB6 - SCL, PB9 - SDA) Alternatif Fonksiyon olarak ayarla
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER9);  // Önce modları temizleyelim
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER9_1);  // Alternatif fonksiyon moduna alalım

    // Pull-up dirençlerini etkinleştir
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR9_0);  // Pull-up modu

    // Yüksek hız modunu ayarla
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR9);  // Yüksek hız

    // Alternatif Fonksiyon 4'ü (AF4) ayarla (I2C1 için)
    GPIOB->AFR[0] |= (4 << (6 * 4));  // PB6 için AF4 ayarla (GPIOB'nin 0. AFR registeri)
    GPIOB->AFR[1] |= (4 << ((9 - 8) * 4));  // PB9 için AF4 ayarla (GPIOB'nin 1. AFR registeri)

    // GPIOD pinlerini LED'ler için ayarlıyoruz (PD14: Kırmızı LED)
    GPIOD->MODER |= GPIO_MODER_MODER14_0;  // PD14 çıkış modu (Kırmızı LED)
}

void I2C1_Init(void)
{
    // I2C'yi başlatma
    I2C1->CR1 &= ~I2C_CR1_PE;  // I2C1'i resetle

    I2C1->CR2 = 16;  // APB1 saat frekansı 16 MHz (saat frekansını giriyoruz)

    // 100 kHz için Clock Control Register (CCR)
    I2C1->CCR = 80;  // I2C frekansını 100 kHz yap (100 kHz için CCR = 80)

    // Maksimum yükselme süresi (1000ns / (1/16MHz) + 1)
    I2C1->TRISE = 17;

    // I2C adresini ayarlama (7-bit adres)
    I2C1->OAR1 = (I2C_ADDRESS << 1);  // STM32'yi 0x52 adresiyle slave olarak ayarla

    I2C1->CR1 |= I2C_CR1_ACK;  // Acknowledge'i (ACK) etkinleştir
    I2C1->CR1 |= I2C_CR1_PE;  // I2C'yi aktif hale getir
}

void UART2_Init(void)
{
    USART2->BRR = 0x0683;  // Baud rate 9600 bps (16MHz clock)
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // UART'ı etkinleştir, TX ve RX'i etkinleştir
}

void UART2_Write(uint8_t data)
{
    while (!(USART2->SR & USART_SR_TXE));  // TXE bayrağının set olmasını bekle
    USART2->DR = data;  // Veriyi gönder
}

// UART string yazma fonksiyonu
void UART2_Write_String(const char* str) {
    while (*str) {
        UART2_Write((uint8_t)*str);
        str++;
    }
}

// Struct verisini UART üzerinden yazdırma
void UART2_Write_Struct(SensorData data) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "ID: %d, Sensor Value: %d, Temperature: %.2f\n",
             data.id, data.sensor_value, data.temperature);
    UART2_Write_String(buffer);
}

// LED kontrol fonksiyonu (Kırmızı LED yanıp söner)
void LED_Blink(void) {
    GPIOD->BSRR = (1 << 14);  // Kırmızı LED'i yak
    delay(500000);            // Kısa bir bekleme
    GPIOD->BSRR = (1 << (14 + 16));  // Kırmızı LED'i söndür
    delay(500000);            // Kısa bir bekleme
}

// Struct ile veriyi al ve gönder
int main(void) {
    System_Clock_Init();
    GPIO_Init();
    I2C1_Init();
    UART2_Init();  // UART2'yi başlat

    SensorData received_data;
    int data_received = 0;  // Veri alındı mı kontrol etmek için

    while (1) {
        // I2C üzerinden veriyi alalım
        uint8_t* data_ptr = (uint8_t*)&received_data;

        // I2C adres eşleşmesini bekle
        if (I2C1->SR1 & I2C_SR1_ADDR) {
            (void)I2C1->SR1;
            (void)I2C1->SR2;  // Adres bayraklarını temizle

            // Struct boyutunda veri al
            for (int i = 0; i < sizeof(SensorData); i++) {
                while (!(I2C1->SR1 & I2C_SR1_RXNE));  // Byte gelmesini bekle
                data_ptr[i] = I2C1->DR;              // Her byte'ı struct'a yaz

                // Son byte geldiyse ACK gönderme, NACK gönder
                if (i == sizeof(SensorData) - 1) {
                    I2C1->CR1 &= ~I2C_CR1_ACK;  // NACK gönder
                }
            }

            // STOP bitini ayarla
            I2C1->CR1 |= I2C_CR1_STOP;

            // Alınan struct verisini UART ile terminale gönder
            UART2_Write_Struct(received_data);
            data_received = 1;  // Veri geldiğini belirt
        } else {
            data_received = 0;  // Veri gelmedi
        }

        // Eğer veri gelmezse kırmızı LED yanıp sönsün
        if (!data_received) {
            LED_Blink();
        }
    }

    return EXIT_SUCCESS;
}

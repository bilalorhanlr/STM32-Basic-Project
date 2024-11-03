#include "stm32f4xx.h"

void SystemClock_Config(void)
{
    // HSI osilatörünü etkinleştir
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)); //kontrol

    FLASH->ACR = FLASH_ACR_LATENCY_0WS; //flash bekleme hızını 0 ayarlıyoruz 16mhz  düşük hız ama pll kullanırsak veya hızı arttırırsak bekleme süresi belirlemek gerekir

    RCC->CFGR |= RCC_CFGR_SW_HSI;  // sistemi hsi de çalış diyoz
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // kontrol

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //ahb1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; //apb1
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // apb2 yi hsi ya göre yapılandırmış oluyoruz
}

void GPIO_Init(void)
{
    // GPIOA ve GPIOD saatlerini etkinleştir
    RCC->AHB1ENR |= (1 << 0); // GPIOA saatini etkinleştir
    RCC->AHB1ENR |= (1 << 3); // GPIOD saatini etkinleştir

    // PA0 pinini dijital giriş olarak yapılandır (buton)
    GPIOA->MODER &= ~(0x3 << (0 * 2));  // PA0 pinini giriş modu olarak ayarla

    // PD12, PD13, PD14, PD15 pinlerini çıkış modu olarak yapılandır (LEDler)
    for (int pin = 12; pin <= 14; pin++) {
        GPIOD->MODER &= ~(0x3 << (pin * 2)); // Önce ilgili bitleri temizle
        GPIOD->MODER |= (0x1 << (pin * 2));  // Sonra çıkış moduna ayarla
        GPIOD->OTYPER &= ~(1 << pin);        // Push-pull olarak ayarla
    }
}

void LED_On(int pin)
{
    GPIOD->BSRR = (1 << pin); // İlgili pini HIGH yap
}

void LED_Off(int pin)
{
    GPIOD->BSRR = (1 << (pin + 16)); // İlgili pini LOW yap
}

void debounce_delay(void)
{
    for (volatile int i = 0; i < 10000; i++); // Basit bir gecikme
}


int main(void)
{
    SystemClock_Config();
    GPIO_Init();
    int num[4] = {12, 13, 14, 15};
    int i = 0;
    int button_released = 1;

    while (1)
    {
        if (GPIOA->IDR & (0x00000001)) // PA0 pinine butona basıldıysa
        {
            if (button_released) // Butonun serbest bırakıldığını kontrol et
            {
                debounce_delay(); // Debounce için kısa bir gecikme

                if (GPIOA->IDR & (0x00000001)) // PA0 hala basılıysa
                {
                    button_released = 0;

                    LED_Off(num[i]);    // Mevcut LED'i söndür
                    i = (i + 1) % 4;    // `i` değerini 0-3 aralığında tut
                    LED_On(num[i]);     // Sıradaki LED'i yak
                }
            }
        }
        else
        {
            button_released = 1; // Buton serbest bırakıldı
        }
    }
}

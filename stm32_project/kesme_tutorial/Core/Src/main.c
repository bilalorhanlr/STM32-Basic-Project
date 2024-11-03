#include "stm32f4xx.h"

#define DEBOUNCE_DELAY 10000

void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    FLASH->ACR = FLASH_ACR_LATENCY_0WS;

    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
}

void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA saatini etkinleştir
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // GPIOD saatini etkinleştir

    // PA0 pinini dijital giriş olarak yapılandır (buton)
    GPIOA->MODER &= ~(0x3 << (0 * 2));  // PA0 pinini giriş modu olarak ayarla

    // PD12 ve PD14 pinlerini çıkış modu olarak yapılandır (LEDler)
    GPIOD->MODER &= ~(0x3 << (12 * 2)); // PD12'yi temizle ve çıkış olarak ayarla
    GPIOD->MODER |= (0x1 << (12 * 2));  // PD12 çıkış modu
    GPIOD->MODER &= ~(0x3 << (14 * 2)); // PD14'ü temizle ve çıkış olarak ayarla
    GPIOD->MODER |= (0x1 << (14 * 2));  // PD14 çıkış modu

    GPIOD->OTYPER &= ~(1 << 12);        // Push-pull olarak ayarla
    GPIOD->OTYPER &= ~(1 << 14);        // Push-pull olarak ayarla
}

void EXTI_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // SYSCFG saatini etkinleştir

    SYSCFG->EXTICR[0] &= ~(0xF);          // EXTI0 hattını PA0 pinine bağla

    EXTI->IMR |= EXTI_IMR_MR0;            // EXTI0 kesmesini etkinleştir
    EXTI->FTSR |= EXTI_FTSR_TR0;          // Düşen kenar tetiklemesini etkinleştir // ek bilgi swıer ile yazılımsal kesme oluşturabiliriz //bazı kesmeler maskelenebilir engellenebilir // kesmeler arasıda öncelik vardır
    EXTI->RTSR |= EXTI_RTSR_TR0;          // Yükselen kenar tetiklemesini etkinleştir düşen veya yükselenden birinde veya low veya high da aktif hale getirebilirsin projene göre

    NVIC_EnableIRQ(EXTI0_IRQn);           // NVIC'de EXTI0 kesmesini etkinleştir
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
    for (volatile int i = 0; i < DEBOUNCE_DELAY; i++); // Basit bir gecikme
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) // EXTI0 hattındaki kesmeyi kontrol et
    {
        debounce_delay(); // Debounce için kısa bir gecikme

        if (GPIOA->IDR & (0x00000001)) // PA0 hala basılıysa
        {
            LED_On(14); // LED'i yak
        }
        else
        {
            LED_Off(14); // LED'i söndür
        }

        EXTI->PR |= EXTI_PR_PR0; // Kesme bayrağını temizle
    }
}

void Timer2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // TIM2 saatini etkinleştir

    TIM2->PSC = 16000 - 1;  // Prescaler: 16000, böylece 1 ms'lik zaman dilimi elde edilir
    TIM2->ARR = 333 - 1;    // Auto-reload: 333 ms'de bir kesme oluşturur (3 kere/saniye)
    TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable
    TIM2->CR1 |= TIM_CR1_CEN;   // Timer'ı başlat

    NVIC_EnableIRQ(TIM2_IRQn);  // NVIC'de TIM2 kesmesini etkinleştir
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) // Update interrupt flag
    {
        TIM2->SR &= ~TIM_SR_UIF; // Kesme bayrağını temizle
        GPIOD->ODR ^= (1 << 12); // PD12 pinini toggle yap
    }
}

int main(void)
{
    SystemClock_Config();
    GPIO_Init();
    EXTI_Init();
    Timer2_Init();

    while (1)
    {
        // Ana döngüde başka işlemler yapılabilir
    }
}

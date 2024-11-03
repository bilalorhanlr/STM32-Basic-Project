#include "stm32f4xx.h"

void SystemClock_Config(void)
{
    RCC->CR |= (1 << 0); // HSI osilatörünü etkinleştir
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR &= ~(3 << 0); // HSI yi sistem saati olarak yapılandırır  -  SW0
    //RCC->CFGR &= ~(1 << 0); // HSI yi sistem saati olarak yapılandırır  -  SW1 üstteki zaten bunu da sıfırlıyo
    while((RCC->CFGR & (3 << 2)) != (0 << 2));
    RCC->AHB1ENR |= (1 << 0); //GPIOA aktif edildi
    RCC->APB1ENR |= (1 << 1); //TIM3 aktif edildi
    RCC->APB2ENR |= (1 << 8); //ADC aktif edildi
    //flash bekleme süresi eklenebilirdi ancak gerek yok
}

void GPIO_Init(void)
{
	//(GPIOx_MODER,GPIOx_OTYPER, GPIOx_OSPEEDR, GPIOx_PUPDR)

	GPIOA->MODER &= ~(3 << (2 * 6)); // PA6'ü alternatif fonksiyona ayarla
	GPIOA->MODER |= (2 << (2 * 6));

	GPIOA->MODER |= (1 << 10);
	GPIOA->MODER |= (1 << 11); // PA5 Analog olarak ayarla

	GPIOA->OTYPER &= ~(1 << 6); // PA6'ü push-pull olarak ayarla  //push pull yada opendrain ayarlar  typer a gerek yok reset hali 0

	GPIOA->OSPEEDR &= ~(3 << (2 * 6)); // PA6'ü hızlı hızda ayarla
	GPIOA->OSPEEDR |= (2 << (2 * 6));

	GPIOA->PUPDR = 0x00000000; // push down yada push up yok

	GPIOA->AFR[0] &= ~(0xF << (4 * 6)); // 4 bit (pa6. pin için) alanı temizler
	GPIOA->AFR[0] |= (0x2 << (4 * 6)); // 0x2 (0010) AF2 değerini yazar

}

void ADC_Init(void)
{
	ADC1->CR1 |= (1 << 24);
	ADC1->CR1 &= ~(1 << 25); //res değerini yani çözünürlük 10 bit oldu yani 1023
	ADC1->CR2 |= (1 << 1); // cont ayarı sürekli dönüşüm yapacak
	ADC1->CR2 |= (1 << 0); // Adon ayarı en aktif etmek için

    ADC1->SMPR2 &= ~(7 << (3 * 5));
    ADC1->SMPR2 |= (4 << (3 * 5)); // Kanal 5 için örnekleme süresini 84 döngü olarak ayarla

    ADC1->SQR1 &= ~(0xF << 20); // L[3:0] bitlerini temizle (1 dönüşüm)
    ADC1->SQR1 |= (0x0 << 20);  // L[3:0] bitlerini 0000 olarak ayarla (1 dönüşüm)
    ADC1->SQR3 &= ~(0x1F << 0); // SQ1 bitlerini temizle
    ADC1->SQR3 |= (5 << 0);     // SQ1 bitlerini 00101 olarak ayarla (Kanal 5)
}

void TIM3_Init(void)
{
    TIM3->PSC = (16 - 1); // Prescaler 16 MHz / 16 = 1 MHz
    TIM3->ARR = (1000 - 1); // ARR (1 ms 1 periyot, %100 duty cycle için 1000)
    TIM3->CCR1 = 0; // Başlangıçta PWM duty cycle %0

	// TIM3 PWM modu
	TIM3->CCMR1 &= ~(3 << 0);   // CC1S = 00, çıkış karşılaştırma modu
	TIM3->CCMR1 |= (1 << 6);
	TIM3->CCMR1 |= (1 << 5);
	TIM3->CCMR1 |= (0x6 << 4);  //PWM MODU 110 için (OC1M)
	TIM3->CCER |= (1 << 0); //ÇIKIŞI EN YAPTIM 1 kanalın

	TIM3->CR1 |= (1 << 0); // Zamanlayıcıyı başlat cen
}

// Motor hızını ayarlayan fonksiyon
void setMotorSpeed(uint16_t pwm_value) {
    TIM3->CCR1 = pwm_value;  // PWM değerini güncelle
}

// UART ile veri okuma fonksiyonu
uint8_t UART2_Read(void) {
    while (!(USART2->SR & USART_SR_RXNE));  // RXNE bayrağının set olmasını bekle
    return USART2->DR;  // Gelen veriyi al
}

// UART2 kurulum fonksiyonu
void UART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // UART2 saatini etkinleştir
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // GPIOA saatini etkinleştir

    GPIOA->MODER |= (2 << 4) | (2 << 6);  // PA2 ve PA3'ü alternatif fonksiyona ayarla
    GPIOA->AFR[0] |= (7 << 8) | (7 << 12);  // PA2 ve PA3 için AF7 (UART2) ayarla

    USART2->BRR = 0x0683;  // Baud rate 9600 bps
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // UART'ı etkinleştir, TX ve RX'i etkinleştir
}

void delay_ms(int delay)
{
	  for(int i = 0; i < delay * 1000; i++);
}

int main(void) {
    SystemClock_Config();
    GPIO_Init();
    TIM3_Init();
    UART2_Init();

    uint8_t mode = 0;
    uint16_t current_pwm = 0;

    while (1) {
        mode = UART2_Read();  // UART'dan veri oku

        if (mode >= '0' && mode <= '9') {  // Geçerli bir mod olup olmadığını kontrol et
            uint16_t target_pwm = (mode - '0') * 1000 / 9;  // Hedef PWM değerini belirle

            // Kademeli olarak motor hızını değiştir
            while (current_pwm != target_pwm) {
                if (current_pwm < target_pwm) {
                    current_pwm++;  // Hızı artır
                } else if (current_pwm > target_pwm) {
                    current_pwm--;  // Hızı azalt
                }
                setMotorSpeed(current_pwm);  // Mevcut PWM değerini ayarla
                delay_ms(1);  // Kademeli değişiklik için küçük bir gecikme
            }
        }
    }
}

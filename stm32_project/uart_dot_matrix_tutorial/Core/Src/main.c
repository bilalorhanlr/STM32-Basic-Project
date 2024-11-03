#include "stm32f4xx.h"
// #include "num.h"


void System_Clock_Init(void)
{
	RCC->CR |= (1<<0); 					//sistem saatini hsi seç
	while(!(RCC->CR & RCC_CR_HSIRDY)); 	//kontrol
	RCC->CFGR &= ~(3 << 0); 			// HSI yi sistem saati olarak yapılandırır  -  SW0
	//RCC->CFGR &= ~(1 << 0); 			// HSI yi sistem saati olarak yapılandırır  -  SW1 üstteki zaten bunu da sıfırlıyo
	while((RCC->CFGR & (3 << 2)) != (0 << 2));
	RCC->AHB1ENR |= (1<<0);
	RCC->AHB1ENR |= (1<<1); //gpio a ve b saatini aktifleştir
	RCC->APB2ENR |= (1<<12); //spi için saat etkinleştir
	//flash bekleme süresini 0 yap DEFAULTTA 0 ELLEMİYOM
}

void GPIO_Init(void)
{
	// 3. PA4, PA5, PA6, PA7 pinlerini alternatif fonksiyon (AF) moduna ayarla
	GPIOA->MODER &= ~((0x3 << (2 * 4))  | (0x3 << (2 * 5)) | (0x3 << (2 * 6)) | (0x3 << (2 * 7)));  // PA4, PA5, PA6, PA7 pinlerini temizle
	GPIOA->MODER |=  ((0x2 << (2 * 4))  | (0x2 << (2 * 5)) | (0x2 << (2 * 6)) | (0x2 << (2 * 7)));  // PA4, PA5, PA6, PA7 pinlerini AF moduna ayarla
	GPIOA->AFR[0] |= (5 << (4 * 4)) | (5 << (4 * 5)) | (5 << (4 * 6)) | (5 << (4 * 7)); // AF5 seç


	// 4. PA1, PA3, PA7, PA8, PA10 pinlerini çıkış (output) moduna ayarla
	GPIOA->MODER &= ~((0x3 << (2 * 1))  | (0x3 << (2 * 3)) | (0x3 << (2 * 7)) | (0x3 << (2 * 8)) | (0x3 << (2 * 10))); // PA1, PA3, PA7, PA8, PA10 pinlerini temizle
	GPIOA->MODER |=  ((0x1 << (2 * 1))  | (0x1 << (2 * 3)) | (0x1 << (2 * 7)) | (0x1 << (2 * 8)) | (0x1 << (2 * 10))); // PA1, PA3, PA7, PA8, PA10 pinlerini output moduna ayarla

	// 5. PB2, PB4, PB5, PB6, PB9, PB11, PB12 pinlerini çıkış (output) moduna ayarla
	GPIOB->MODER &= ~((0x3 << (2 * 2))  | (0x3 << (2 * 4)) | (0x3 << (2 * 5)) | (0x3 << (2 * 6)) | (0x3 << (2 * 9)) | (0x3 << (2 * 11)) | (0x3 << (2 * 12))); // PB2, PB4, PB5, PB6, PB9, PB11, PB12 pinlerini temizle
	GPIOB->MODER |=  ((0x1 << (2 * 2))  | (0x1 << (2 * 4)) | (0x1 << (2 * 5)) | (0x1 << (2 * 6)) | (0x1 << (2 * 9)) | (0x1 << (2 * 11)) | (0x1 << (2 * 12))); // PB2, PB4, PB5, PB6, PB9, PB11, PB12 pinlerini output moduna ayarla

}

void SPI1_Init(void)
{
	SPI1->CR1 &= ~(1 << 0);		//	CPHA (Saat Fazı): Verinin saat sinyalinin ilk veya ikinci kenarında örnekleneceğini belirler.
	SPI1->CR1 &= ~(1 << 1);		//	CPOL (Saat Kutbu): Veri örnekleme ve veri iletimi sırasında saat sinyalinin hangi seviyede olacağını belirler.
	SPI1->CR1 &= ~(1 << 2);		//	MSTR (Master/Slave Seçimi): SPI'nin master veya slave modunda çalışıp çalışmayacağını ayarlar.
	SPI1->CR1 &= ~(0x7 << 3);  	// 0x7 = 0b111, 3. bit kaydırma yaparak 0b111000 ile maskelenir ve sıfırlanır
	SPI1->CR1 |= (0x3 << 3);		//	BR (Baud Rate): SPI hızını ayarlar.
	SPI1->CR1 &= ~(1 << 11);		//	DFF (Data Frame Format): 8-bit veya 16-bit veri çerçeve formatını seçer.
}

void UART_Init(void)
{
	 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // UART2 saatini etkinleştir

	 GPIOA->MODER |= (2 << 4) | (2 << 6);  // PA2 ve PA3'ü alternatif fonksiyona ayarla
	 GPIOA->AFR[0] |= (7 << 8) | (7 << 12);  // PA2 ve PA3 için AF7 (UART2) ayarla

	 USART2->BRR = 0x0683;  // Baud rate 9600 bps
	 USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // UART'ı etkinleştir, TX ve RX'i etkinleştir
}

void UART2_Write(uint8_t data) {
    while (!(USART2->SR & USART_SR_TXE));  // TXE bayrağının set olmasını bekle
    USART2->DR = data;  // Veriyi gönder
}

int Spi_Recieve(void)
{
	if(SPI1->SR & SPI_SR_RXNE)
		return SPI->DR;
	return 0;
}

void LED_On(uint16_t pin) {
    GPIOD->BSRR = (1 << pin); // İlgili pini HIGH yap
}

void LED_Off(uint16_t pin) {
    GPIOD->BSRR = (1 << (pin + 16)); // İlgili pini LOW yap
}

void delay_ms(int delay) {
    for(int i = 0; i < delay * 1000; i++);
}

int main(void)
{
    System_Clock_Init();
    GPIO_Init();
    SPI1_Init();
    UART2_Init();

    int numr_new = 0;
    while(1)
    {
        if(SPI1->SR & SPI_SR_RXNE) // Verinin geldiğini kontrol et
        {
            numr_new = Spi_Recieve(); // Veriyi al
            UART2_Write(numr_new); // Alınan veriyi UART üzerinden gönder
        }
    }
}


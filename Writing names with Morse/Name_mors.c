#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>

#define  GPIO_NOPULL        0x00000000U /*!< No Pull-up or Pull-down activation */
#define  GPIO_PULLUP        0x00000001U /*!< Pull-up activation                 */
#define  GPIO_PULLDOWN      0x00000002U /*!< Pull-down activation               */

int state;
UART_HandleTypeDef huart2;
char a[26][5];
char alpha[27];
char harf[6];
char kelime[100];
char name_surname[100][100];
int kelime_sayisi = 0;
int harf_sayisi = 0;
int print_count = 0;

typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins.
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDeff;

void Init_OnBoard_LEDs(void);
void configure_Button(void);

void Init_OnBoard_LEDs(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE(); //Enable clock to GPIOD
	GPIO_InitTypeDeff BoardLEDs; // declare a variable of type struct GPIO_InitTypeDef
	BoardLEDs.Mode = GPIO_MODE_OUTPUT_PP; // set pin mode to output
	BoardLEDs.Pin = GPIO_PIN_15; // select pins PD12-PD15
	HAL_GPIO_Init(GPIOD, &BoardLEDs); // initialize PD12-PD15 pins by passing port name and address of BoardLEDs struct
}

void configure_Button(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Enable clock to GPIOA
	GPIO_InitTypeDeff PushButton;  // declare a variable of type struct GPIO_InitTypeDef
	PushButton.Mode = GPIO_MODE_INPUT; // set pin mode to input
	PushButton.Pin = GPIO_PIN_0;  // select pin PA0 only
	PushButton.Pull = GPIO_NOPULL; // set no internal pull-up or pull-down resistor
	HAL_GPIO_Init(GPIOA, &PushButton); //  initialize PA0 pins by passing port name and address of PushButton struct
}


void UART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    printf("uart2 hata veriyor");
  }
}

void UART2_Send(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart2, pData, Size, HAL_MAX_DELAY);
}

void str_up(int i, char *str)
{
    int j = 0;
    while(str[j])
    {
        a[i][j] = str[j];
        j++;
    }
    a[i][j] = '\0';
}

void str_up_name(char *str,int i)
{
    int j = 0;
    while(str[j])
    {
        name_surname[i][j] = str[j];
        j++;
    }
    name_surname[i][j] = '\0';
}

void str_up2(char *str)
{
    int j = 0;
    while(str[j])
    {
        alpha[j] = str[j];
        j++;
    }
    alpha[j] = '\0';
}

void init_st()
{
    str_up2(" abcdefghijklmnoprqrstuvwxyz");
    str_up(0   ,"0");
    str_up(1   ,"210");
    str_up(2   ,"12220");
    str_up(3   ,"12120");
    str_up(4   ,"1220");
    str_up(5   ,"20");
    str_up(6   ,"22120");
    str_up(7   ,"1120");
    str_up(8   ,"22220");
    str_up(9   ,"220");
    str_up(10   , "21110");
    str_up(11   , "1210");
    str_up(12   , "2122");
    str_up(13   , "110");
    str_up(14   , "120");
    str_up(15   , "1110");
    str_up(16   , "21120");
    str_up(17   , "11210");
    str_up(18   , "2120");
    str_up(19   , "2220");
    str_up(20   , "10");
    str_up(21   , "2210");
    str_up(22   , "22210");
    str_up(23   , "2110");
    str_up(24   , "1210");
    str_up(25   , "12110");
    str_up(26   , "11220");
}

int time0_state(int state)
{
    long long int i = 0;
    while (state == 0)
    {
        state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        i++;
    }
    return i;
}


int time1_state(int state)
{
    long long int i = 0;
    print_count = 0;
    while (state == 1)
    {
        state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        i++;
    }
    return i;
}

int str_equ_str(char *str1,char *str2)
{
    int i = 0;
    while(str1 && str2 && str1[i] && str2[i] && str1[i] == str2[i])
        i++;
    if(str1[i] == '\0' && str2[i] == '\0')
        return(1);
    else
        return(0);
}

void led_start(char *isim_soyisim)
{
    int i = 0;
    int j = 0;
    int h = 0;

    while(isim_soyisim[i])
    {
        j = 0;
        while(j <= 26 )
        {
            h = 0;
            if(isim_soyisim[i] == alpha[j])
            {
                while(a[j][h])
                {
                    if (a[j][h] == '2')
                    {
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
                        long int m = 0;
                        while(m < 200000)
                            m++;
                        m = 0;
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
                        while(m < 200000)
                            m++;
                        m = 0;
                    }
                    else if (a[j][h] == '1')
                    {
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
                        long int m = 0;
                        while(m < 700000)
                            m++;
                        m = 0;
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
                        while(m < 700000)
                            m++;
                        m = 0;
                    }
                    else if (isim_soyisim[i] == ' ')
                    {
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
                        long int m = 0;
                        while(m < 1500000)
                            m++;
                        m = 0;
                    }
                    h++;
                }
            }
            j++;
            long int m = 0;
            while(m < 200000)
                m++;
            m = 0;
        }
        i++;
    }
}

int main(void)
{
    int time0;
    int time1;
    char kisa = '2';
    char uzun = '1';
    int i = 0;
    int k = 0;

    HAL_Init();
    UART2_Init();
    init_st();
    Init_OnBoard_LEDs(); 
    configure_Button(); 

    char *isim_soyisim = "ozgur gunes";
    if (*isim_soyisim)
        led_start(isim_soyisim);
    else
    {
        while(1)
        {
            time1 = 0;
            time0 = 0;

            state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
            if(state == 1)
                time1 = time1_state(state);
            else
                time0 = time0_state(state);
            if (time0)
            {
                if(time0 > 5*300000 && time0 < 10*300000) 
                {
                   
                    harf[i] = '0';
                    harf[++i] = '\0';
                    i = 0;
                    for(int r = 0; r < 26;r++)
                    {
                        if(str_equ_str(harf,a[r]))
                        {
                            kelime[k] = alpha[r];
                            k++;
                        }
                    }
                    kelime[k] = '\0';
                }
                else if(time0 > 10*300000 && time0 < 20*300000) 
                {
                    harf[i] = '0';
                    harf[++i] = '\0';
                    i = 0;
                    for(int y = 0; y <= 26;y++)
                    {
                        if(str_equ_str(harf,a[y]))
                        {
                            kelime[k] = alpha[y];
                            k++;
                        }
                    }
                    kelime[k++] = ' ';
                    kelime[k] = '\0';
                    str_up_name(kelime,kelime_sayisi);
                    kelime_sayisi++;
                    k = 0;
                    while(kelime[k] != 0)
                    {
                        kelime[k] = '\0';
                        k++;
                    }
                    k = 0;
                }
                else if( time0 > 20*300000 && print_count == 0) 
                {
                    harf[i] = '0';
                    harf[++i] = '\0';
                    i = 0;
                    for(int s = 0; s <= 26;s++)
                    {
                        if(str_equ_str(harf,a[s]))
                        {
                            kelime[k] = alpha[s];
                            k++;
                        }
                    }
                    int b = 0;
                    while(kelime_sayisi > b)
                    {
                        printf("%s",name_surname[b]);
                        int harf = 0;
                        while(name_surname[b][harf])
                        {
                            name_surname[b][harf++] = 0;
                        }
                    }
                    int top = 0;
                    int q;
                    int w;
                    while (name_surname[q][w])
                    {

                        if (!(name_surname[q][w]))
                        {
                            q++;
                            w = 0;
                        }
                        if (name_surname[q][w])
                            top += name_surname[q][w];
                        w++;
                    }
                    UART2_Send(top, sizeof(int));
                    print_count++;
                }
            }
            else if (time1)
            {
                if (time1 < 300000)
                {
                    harf[i++] = kisa;
                }
                else
                {
                    harf[i++] = uzun;
                }
            }
        }
    }
}
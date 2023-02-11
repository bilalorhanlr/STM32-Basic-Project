#include "stm32f4xx.h"
#include <stdlib.h>
#include <string.h>

void delay(uint32_t time)
{
	while (time--);  
}

int ascii_mod(char *strs)
{
    int i;
    int k;
    int a;

    i = 0;
    k = 0;
    while(strs[i])
        k += strs[i++];
    a = k % 4;
    return (a);
}

int witch_colour(char *asd)
{
    int red;
    int blue;
    int green;
    int orange;
    int mod;

    green = 0x00001000;
    orange = 0x00002000;
    blue = 0x00008000;
    red = 0x00004000;                        
    mod = ascii_mod(asd);
    if (mod == 3)
        return green;
    else if (mod == 2)
        return orange;
    else if (mod == 1)
        return blue;
    else if (mod == 0)
        return red;
}

void init_leds()
{
    RCC->AHB1ENR = 0x00000008;

    GPIOD->MODER = 0x55000000;
    GPIOD->OTYPER = 0x00000000;
    GPIOD->OSPEEDR = 0xFF000000;
    GPIOD->PUPDR = 0x00000000;
}

void turn_name_on(char *name)
{
    int colour;

    colour = witch_colour(name);
    GPIOD->ODR = colour;
}

void turn_surname_on(char *surname)
{
    int colour;

    colour = witch_colour(surname);
    GPIOD->ODR = colour;
}

void turn_name_of(char *name)
{
    int colour;

    colour = witch_colour(name);
    GPIOD->ODR = 0x00000000;
}

void turn_surname_of(char *surname)
{
    int colour;

    colour = witch_colour(surname);
    GPIOD->ODR = 0x00000000;
}

int main()
{
    char *name;
    char *surname;
    int i;

    name = (char *) malloc(sizeof(char) * 30);
    surname = (char *) malloc(sizeof(char) * 20);
    name = "ozgur";
    surname = "gunes";
    init_leds();
    while (1)
    {
        i = 0;
        while (strlen(name) > i)
        {
            turn_name_on(name);
            delay(1000);
            turn_name_of(name);
            delay(1000);
            i++;
        }
        i = 0;
        while (strlen(surname) > i)
        {
            turn_name_on(surname);
            delay(1000);
            turn_name_of(surname);
            delay(1000);
            i++;
        }
    }
}

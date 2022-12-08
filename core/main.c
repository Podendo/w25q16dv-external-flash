#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>

#include "include/w25q16bv.h"

static volatile uint32_t sys_millis;



void sleep_ms(uint32_t delay_millis);


static void clock_setup(void);
static void gpio_setup(void);
static void systick_setup(void);

void systick_handler(void);



int main(void)
{
    printf("\nCheck makefile\n");
    check_flash_module();

    clock_setup();
    gpio_setup();
    systick_setup();

    while(1)
    {
        //gpio_toggle(GPIOD, GPIO12 | GPIO13);
        //sleep_ms(500);
        //if(gpio_get(GPIOA, GPIO0)){
        //   sleep_ms(500);
        //}
        //gpio_toggle(GPIOD, GPIO14 | GPIO15);

    }


    return 0;
}




void sleep_ms(uint32_t delay_millis)
{
    uint32_t current_ms = sys_millis;

    while((sys_millis - current_ms) < delay_millis){
        continue;
    }

    return;
}


static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOA);
    //rcc_periph_clock_enable(RCC_SPI2);
    //rcc_periph_clock_enable(RCC_USART2);
    return;
}


static void gpio_setup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_clear(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO12 | GPIO13 | GPIO14 | GPIO15);

    return;
}
static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(2100 - 1);
    systick_counter_enable();
    systick_interrupt_enable();

    return;
}

void sys_tick_handler(void)
{
    sys_millis += 1;
    return;
}

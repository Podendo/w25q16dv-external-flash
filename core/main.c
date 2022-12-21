#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>

#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/spi.h>

#include <libopencm3/stm32/f4/nvic.h>

#include "include/w25q16bv.h"

//#define STM_DISCO
#define STM_BLACK

extern volatile uint32_t flash_sys_ms;

static volatile uint32_t sys_millis;
static volatile uint8_t usart_rx_flag;


static  uint8_t usart_rx_buffer[128];
static  uint8_t spi_rx_buffer[128];

static volatile uint8_t urxb = 0;
static volatile uint8_t srxb = 0;


extern void sleep_ms(uint32_t delay_millis);

static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);

static void usart_setup(void);
static void spi_setup(void);
//void sys_tick_handler(void);
void led_blinking(void);

void usart2_isr(void);
void spi1_isr(void);

void usart_transmit(uint32_t USARTx, uint8_t *data, uint8_t size);
void spi_transmit(uint32_t SPIx, uint8_t *data, uint8_t size);

/*#######################################################*/

int main(void)
{

    clock_setup();
    gpio_setup();
    spi_setup();
    usart_setup();
    systick_setup();

    w25q_power_dwn();

    static uint16_t pack = 0x0000;
    uint8_t page_data[12] = {'l', 'o', 'r', 'e', 'm', ' ', 'i', 'p', 's', 'u', 'm', '\n'};

    struct w25q_sr1 sr1;
    struct w25q_sr2 sr2;

    sr1.protect_1 = W25Q_SR1_UNPROTECT;

    sr1.topbtm_protect = W25Q_SR1_SEC_TB_NPRT;

    sr1.block_protect = W25Q_SR1_BLCK_NPRT_2 |   \
                        W25Q_SR1_BLCK_NPRT_1 |   \
                        W25Q_SR1_BLCK_NPRT_0;    \

    sr2.quad_spi = W25Q_SR2_QUAD_DS;
    sr2.protect_2 = W25Q_SR2_UNPROTECT;

    w25q_power_dwn_release();

    //w25q_erase_field(0x00, W25Q_ERASE_CHIP_TYPE);
    //w25q_write_status_reg(&sr1, &sr2);

    spi_rx_buffer[0] = w25q_read_status_reg(W25Q_SR1);
    spi_rx_buffer[1] = w25q_read_status_reg(W25Q_SR2);

    usart_transmit(USART2, spi_rx_buffer, 2);

    //w25q_power_dwn();
    uint8_t device_uid[8];

    while(1)
    {

#ifdef STM_DISCO

        if(gpio_get(GPIOA, GPIO0)){

            spi_transmit(SPI1, pack, 7);
            gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
            usart_transmit(USART2, spi_rx_buffer, 7);
            gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
            sleep_ms(500);
        }
        else{
            __asm__("nop");
        }
#endif

#ifdef STM_BLACK    

        if(gpio_get(GPIOA, GPIO0)){
            w25q_power_dwn_release();
            w25q_erase_field(0x000000, W25Q_ERASE_SECTOR_TYPE);
        }

        w25q_read_unique_id(device_uid);
        //usart_transmit(USART2, device_uid, 8);
        
        pack = w25q_read_manufacturer();
        spi_rx_buffer[0] = pack >> 8;
        spi_rx_buffer[1] = pack >> 0;
        //usart_transmit(USART2, spi_rx_buffer, 2);

        w25q_page_program(0x00, page_data, 12);

        w25q_read_data(0x00, spi_rx_buffer, 12);

        usart_transmit(USART2, spi_rx_buffer, 12);

#endif

        led_blinking();

        sleep_ms(1000);
        
    }

    return 0;
}

/*#######################################################*/

/* ________________ FUNCTION PROTOTYPES ________________ */

extern void sleep_ms(uint32_t delay_millis)
{
    uint32_t current_ms = sys_millis;
    while((sys_millis - current_ms) < delay_millis){
        continue;
    }
    return;
}

/*_______________________________________________________*/

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOD);

    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_USART2);

    return;
}

/*_______________________________________________________*/

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(21000 - 1);
    systick_counter_enable();
    systick_interrupt_enable();

    return;
}

/*_______________________________________________________*/

static void gpio_setup(void)
{

#ifdef STM32_DISCO
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_clear(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
            GPIO12 | GPIO13 | GPIO14 | GPIO15);
#endif

#ifdef STM_BLACK
    gpio_clear(GPIOA, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_clear(GPIOA, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

#endif

    return;
}

/*_______________________________________________________*/

/**
 *  Configure peripherals: SCK -> PB3,
 * MOSI -> PB5, MISO -> PB4, CS -> PA15
*/
static void spi_setup(void)
{
    /* NVIC spi enable interrupts */
    //nvic_enable_irq(NVIC_SPI1_IRQ);

    /* Chip select configuration: */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
    gpio_set(GPIOA, GPIO15);
    /* SPI-1 CLK configuration: */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    /* SPI-1 MISO configuration: */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
    /* SPI-1 MOSI configuration: */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);

    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO3);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO4);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO5);
    
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO15);

    gpio_set_af(GPIOB, GPIO_AF5, GPIO3);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO4);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO5);

    /**
     * Set up SPI in Master mode with:
     * clock baud rate: 1/4 of peripheral clock frequency
     * clock polarity: idle low
     * clock phase: data valid on falling edge
     * data frame format: 8-bit
     * frame format: MSB first
    */

    /* Polarity CPHA=1, CPOL=0. !!! RM page 879 */
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256,   \
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,        \
                    SPI_CR1_CPHA_CLK_TRANSITION_1,          \
                    SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);    \
    
    /**
     * Set /CS management to software. Note, that nss pin must be set to high,
     * otherwise the spi peripheral will not send any data out.
    */
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    /* Enable SPI1 Receive interrupt */
    //spi_enable_rx_buffer_not_empty_interrupt(SPI1);
    /* Enable SPI1 Transmit interrput */
    //spi_enable_tx_buffer_empty_interrupt(SPI1);
    /* Enable SPI1 periph */
    spi_enable(SPI1);

    return;
}

/*_______________________________________________________*/

void spi_transmit(uint32_t SPIx, uint8_t *data, uint8_t size)
{
    for(int i = 0; i < size; i++){
        spi_send(SPIx, data[i]);
    }
    return;
}

/*_______________________________________________________*/

static void usart_setup(void)
{
    /* Enable the USART2 interrupt */
    nvic_enable_irq(NVIC_USART2_IRQ);

    /* Setup GPIO pin for USART2 Tx */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    /* Setup GPIO pin for USART2 Rx */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);
    /* Setup USART2 Tx pin as alternate function */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
    /* Setup USART2 Rx pin as alternate function */
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

    /* Setup USART2 parameters */
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);

    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    /* Enable USART2 receive interrupt */
    usart_enable_rx_interrupt(USART2);
    /* Enable USART2 transmit interrupt */
    //usart_enable_tx_interrupt(USART2);
    /* Enable USART2 in Tx-Rx mode (PA2 - Tx; PA3 - Rx) */
    
    usart_enable(USART2);

    return;
}

/*_______________________________________________________*/

void usart_transmit(uint32_t USARTx, uint8_t *data, uint8_t size)
{
    for(int i = 0; i < size; i++){
        usart_send_blocking(USARTx, data[i]);
    } 
    return;
}

/*_______________________________________________________*/

void usart2_isr(void)
{
    uint32_t usart_register = 0;

    usart_register = USART_SR(USART2);
    if(usart_register & USART_SR_RXNE){
        usart_rx_buffer[urxb] = USART_DR(USART2);
        if(urxb == 128) urxb = 0;

        urxb += 1;
        usart_rx_flag = 1;
    }

    return;
}

/*_______________________________________________________*/

/*
void spi1_isr(void)
{
    uint32_t spi_register = 0;

    spi_register = SPI_SR(SPI1);
        if(spi_register & SPI_SR_RXNE){
            spi_rx_buffer[srxb] = SPI_DR(SPI1);
            if(srxb == 128) srxb = 0;
            srxb += 1;
        }

    return;
}
*/

/*_______________________________________________________*/

void sys_tick_handler(void)
{
    sys_millis += 1;
    flash_sys_ms += 1;
    return;
}

/*_______________________________________________________*/

void led_blinking(void)
{
#ifdef STM_DISCO
    gpio_toggle(GPIOD, GPIO12 | GPIO13);
    sleep_ms(500);
    if(gpio_get(GPIOA, GPIO0)){
       sleep_ms(500);
    }
    gpio_toggle(GPIOD, GPIO14 | GPIO15);
#endif

#ifdef STM_BLACK
    gpio_toggle(GPIOA, GPIO1);
    sleep_ms(500);
    if(gpio_get(GPIOA, GPIO0)){
        sleep_ms(500);
    }

#endif
}

/* ______________ END FUNCTION PROTOTYPES ______________ */


/* END OF FILE */
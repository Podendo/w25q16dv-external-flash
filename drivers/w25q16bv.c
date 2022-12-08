#include "include/w25q16bv.h"

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/nvic.h>


#define FLASH_SPI              SPI1
#define FLASH_NSS_PORT         GPIOA
#define FLASH_NSS_PIN          GPIO15

#define W25Q_MAX_DELAY_MS      0x0F

#define W25Q_SELECT()          gpio_clear(FLASH_NSS_PORT, FLASH_NSS_PIN)
#define W25Q_UNSELECT()        gpio_set(FLASH_NSS_PORT, FLASH_NSS_PIN)

extern volatile uint32_t flash_sys_ms = 0;

void w25q_send_byte(uint8_t data)
{
    spi_send(FLASH_SPI, data);

    return;
}


uint8_t w25q_read_byte(void)
{
    uint8_t data = 0x00;
    uint32_t msec = flash_sys_ms;

    //data = spi_read(FLASH_SPI);
    do{
        if((SPI_SR(FLASH_SPI) & SPI_SR_RXNE) != 0){
            data = SPI_DR(FLASH_SPI);
            gpio_toggle(GPIOD, GPIO14);
            return data;
        }

    } while((flash_sys_ms - msec) < W25Q_MAX_DELAY_MS);

    return data;
}


void w25q_write_enable(void)
{
    W25Q_SELECT();
    w25q_send_byte(W25Q_WRITE_ENA);
    W25Q_UNSELECT();

    return;
}


void w25q_write_disable(void)
{
    W25Q_SELECT();
    w25q_send_byte(W25Q_WRITE_DIS);
    W25Q_UNSELECT();

    return;
}


uint8_t w25q_status_reg_read(uint8_t SRx)
{
    uint8_t reg_bits = 0;

    w25q_write_enable();
    W25Q_SELECT();

    switch (SRx)
    {
    case W25Q_SR1:
        w25q_send_byte(W25Q_READ_SR1);
        break;
    case W25Q_SR2:
        w25q_send_byte(W25Q_READ_SR2);
        break;
    default:
        break;
    }

    reg_bits = w25q_read_byte();
    w25q_write_disable();

    return reg_bits;
}


/* ToDo: Still don't understand the procedure of executing this func in  ds 

void w25q_status_reg_write(uint8_t SRx, uint8_t reg_bits)
{
    w25q_write_enable();
    W25Q_SELECT();

    switch (SRx)
    {
    case SR1:
        break;
    case SR2:
        break;
    default:
        break;
    }

    w25q_write_disable();

    return;
}
*/

void w25q_read_data(uint32_t address, uint8_t *databuf, uint8_t buff_size)
{
    uint8_t byte = 0;

    w25q_write_enable();
    W25Q_SELECT();

    w25q_send_byte(W25Q_READ_DATA);

    w25q_send_byte((address >> 16) & 0xFF);
    w25q_send_byte((address >> 8) & 0xFF);
    w25q_send_byte((address >> 0) & 0xFF);

    for(byte = 0; byte < buff_size; byte++){
        databuf[byte] = w25q_read_byte();
    }

    w25q_write_disable();

    return;
}


void w25q_page_program(uint32_t address, uint8_t *databuf, uint8_t buff_size)
{
    w25q_write_enable();
    W25Q_SELECT();

    w25q_send_byte(W25Q_PAGE_PRG);

    w25q_send_byte((address >> 16) & 0xFF);
    w25q_send_byte((address >> 8) & 0xFF);
    w25q_send_byte((address >> 0) & 0xFF);

    for(uint8_t byte = 0; byte < buff_size; byte++){
        w25q_send_byte(databuf[byte]);
    }

    w25q_write_disable();
    return;
}

uint16_t w25q_read_manufacturer(void)
{
    uint16_t device =0x0000;

    w25q_write_enable();
    W25Q_SELECT();

    w25q_send_byte(W25Q_ID_MANF);

    w25q_send_byte(0x00);
    w25q_send_byte(0x00);
    w25q_send_byte(0x00);

    device = device | w25q_read_byte() << 8;
    device = device | w25q_read_byte();

    w25q_write_disable();

    return device;
}


/* END OF FILE */
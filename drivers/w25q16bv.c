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

#define W25Q_MAX_DELAY_MS      0xFF

#define W25Q_SELECT()          gpio_clear(FLASH_NSS_PORT, FLASH_NSS_PIN)
#define W25Q_UNSELECT()        gpio_set(FLASH_NSS_PORT, FLASH_NSS_PIN)

extern volatile uint32_t flash_sys_ms = 0;

extern void sleep_ms(uint32_t delay_millis);


uint8_t w25q_handler(void)
{
    usart_send_blocking(USART2, 0xFF);
    gpio_toggle(GPIOA, GPIO1);

    return;
}

/* Works correct */
uint8_t w25q_transfer_byte(uint8_t data)
{
    uint32_t timeout = flash_sys_ms;

    while((SPI_SR(FLASH_SPI) & SPI_SR_TXE) == 0){
        if((flash_sys_ms - timeout) > W25Q_MAX_DELAY_MS)
            return w25q_handler();
    }
    spi_send(FLASH_SPI, data);
    while((SPI_SR(FLASH_SPI) & SPI_SR_RXNE) == 0){
        if((flash_sys_ms - timeout) > W25Q_MAX_DELAY_MS)
            return w25q_handler();
    }

    return (uint8_t)SPI_DR(FLASH_SPI);
}

/* Works correct */
void w25q_write_enable(void)
{
    W25Q_SELECT();
    w25q_transfer_byte(W25Q_WRITE_ENA);
    W25Q_UNSELECT();

    return;
}

/* Works correct */
void w25q_write_disable(void)
{

    W25Q_SELECT();
    w25q_transfer_byte(W25Q_WRITE_DIS);
    W25Q_UNSELECT();

    return;
}

/* Works correct */
uint8_t w25q_status_reg_read(uint8_t SRx)
{
    uint8_t reg_bits = 0x00;
    W25Q_SELECT();

    switch (SRx)
    {
    case W25Q_SR1:
        w25q_transfer_byte(W25Q_READ_SR1);
        reg_bits = w25q_transfer_byte(0x00);
        break;
    case W25Q_SR2:
        w25q_transfer_byte(W25Q_READ_SR2);
        reg_bits = w25q_transfer_byte(0x00);
        break;
    default:
        break;
    }

    W25Q_UNSELECT();

    return reg_bits;
}

/* Works correct */
void w25q_status_reg_write(struct w25q_sr1 *sr1, struct w25q_sr2 *sr2)
{
    uint8_t frame_sr1 = 0x00;
    uint8_t frame_sr2 = 0x00;

    w25q_write_enable();
    W25Q_SELECT();

    frame_sr1 |= sr1->protect_1;
    frame_sr1 |= sr1->sector_protect;
    frame_sr1 |= sr1->topbtm_protect;
    frame_sr1 |= sr1->block_protect;

    frame_sr1 |= sr1->latch;
    frame_sr1 |= sr1->status;

    frame_sr2 |= sr2->suspend;
    frame_sr2 |= sr2->quad_spi;
    frame_sr2 |= sr2->protect_2;

    w25q_transfer_byte(W25Q_WRITE_SR);

    w25q_transfer_byte(frame_sr1);
    w25q_transfer_byte(frame_sr2);


    W25Q_UNSELECT();
    w25q_write_disable();

    return;
}


/* ??? */
void w25q_read_data(uint32_t address, uint8_t *databuf, uint8_t buff_size)
{

    w25q_write_enable();
    W25Q_SELECT();

    w25q_transfer_byte(W25Q_READ_DATA);

    w25q_transfer_byte((address >> 16) & 0xFF);
    w25q_transfer_byte((address >> 8) & 0xFF);
    w25q_transfer_byte((address >> 0) & 0xFF);
    do{
        *databuf = w25q_transfer_byte(0x00);
        databuf++;
        buff_size--;
    } while(buff_size > 0x00);

    W25Q_UNSELECT();
    w25q_write_disable();

    return;
}

/* ??? */
void w25q_page_program(uint32_t address, uint8_t *databuf, uint8_t buff_size)
{
    w25q_write_enable();
    W25Q_SELECT();

    w25q_transfer_byte(W25Q_PAGE_PRG);

    w25q_transfer_byte((address >> 16) & 0xFF);
    w25q_transfer_byte((address >> 8) & 0xFF);
    w25q_transfer_byte((address >> 0) & 0xFF);
    do{
        w25q_transfer_byte(*databuf);
        databuf++;
        buff_size--;
    } while(buff_size > 0x00);

    W25Q_UNSELECT();
    w25q_write_disable();

    return;
}


/* Works correct */
uint16_t w25q_read_manufacturer(void)
{
    uint16_t device =0x0000;
    uint8_t bytes[2] = {0x00, 0x00};

    W25Q_SELECT();

    w25q_transfer_byte(W25Q_ID_MANF);

    w25q_transfer_byte(0x00);
    w25q_transfer_byte(0x00);
    w25q_transfer_byte(0x01);

    bytes[0] = w25q_transfer_byte(0x00);
    bytes[1] = w25q_transfer_byte(0x00);

    W25Q_UNSELECT();

    device = ((device | bytes[0]) << 8) | ((device | bytes[1]) << 0);

    return device;
}


/* Works correct */
void w25q_power_dwn_release(void)
{
    W25Q_SELECT();

    w25q_transfer_byte(W25Q_PWR_DWN_RELEASE);

    W25Q_UNSELECT();

    return;
}


/* Works Correct */
void w25q_power_dwn(void)
{
    W25Q_SELECT();

    w25q_transfer_byte(W25Q_PWR_DWN);

    W25Q_UNSELECT();

    return;
}


/* END OF FILE */
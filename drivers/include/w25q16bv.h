#ifndef W25Q16BV_H
#define W25Q16BV_H

#define FLASH_H "W25Q16BV_HEADER"

/* INSTRUCTIONS SET OF THE W25Q16BV */
#define W25Q_ID_DEVICE          0x14

#define W25Q_WRITE_ENA          0x06
#define W25Q_WRITE_DIS          0x04

#define W25Q_READ_SR1           0x05
#define W25Q_READ_SR2           0x35
#define W25Q_WRITE_SR           0x01
#define W25Q_PAGE_PRG           0x02

#define W25Q_QUAD_PAGE_PRG      0x32

#define W25Q_SECTOR_ERASE       0x20    // 04 KB 
#define W25Q_BLOCK_ERASE_32     0x52    // 32 KB
#define W25Q_BLOCK_ERASE_64     0xD8    // 64 KB
#define W25Q_CHIP_ERASE         0xC7

#define W25Q_ERASE_SUSPEND      0x75
#define W25Q_ERASE_RESUME       0x7A

#define W25Q_PWR_DWN            0xB9
#define W25Q_CONT_RMODE_R       0xFF


#define W25Q_READ_DATA          0x03
#define W25Q_READ_FAST          0x0B
#define W25Q_READ_FAST_DO       0x3B
#define W25Q_READ_FAST_DIO      0xBB
#define W25Q_READ_QUAD_O        0x6B
#define W25Q_READ_QUAD_IO       0xEB
#define W25Q_READ_WORD          0xE7    // QUAD-SPI
#define W25Q_READ_OCTAL         0xE3    // QUAD-SPI


#define W25Q_PWR_DWN_RELEASE    0xAB
#define W25Q_ID_MANF            0x90
#define W25Q_ID_MANF_D          0x92    // DUAL-SPI
#define W25Q_ID_MANF_Q          0x94    // QUAD-SPI
#define W25Q_ID_JEDEC           0x9F
#define W25Q_READ_UID           0x4B


#define W25Q_BLOCK_MAX_ADDR     0x1FFFFF
#define W25Q_BLOCK_ZER_ADDR     0x000000

#define W25Q_CLOCK_FREQ_MAX     0x50    // 80 MHz
#define W25Q_CLOCK_FREQ_AVG     0x32    // 50 MHz

#define W25Q_SR1                     0x01
#define W25Q_SR2                     0x02

/* MACRO FUNCTIONS FOR SETTING ADDRESSES*/
#define SET_BLOCK(BLOCK)                         \
        ((W25Q_BLOCK_ZER_ADDR | BLOCK) << 16)    

#define SET_SECTOR(SECTOR)                       \
        ((W25Q_BLOCK_ZER_ADDR | SECTOR) << 12)    

#define W25Q_SET_ADDR(BLOCK, SECTOR)             \
        ((W25Q_BLOCK_ZER_ADDR | SET_BLOCK(BLOCK) | SET_SECTOR(SECTOR)))


/* STANDART INCLUDES */
#include "inttypes.h"

extern volatile uint32_t flash_sys_ms;

/**
 * Send one byte using libopencm3 spi functions. Initialization of
 * this SPI must be done in main.c file according to ds parameters.
 * Before using this function you need to SELECT/UNSELECT NSS pin
 * with macro functions (in flash source file).
*/
void w25q_send_byte(uint8_t data);
/* Done, checked */

/**
 * This function returns the output of SPIx_DR using libopencm3 lib
 * spi_read function.
*/
uint8_t w25q_read_byte(void);
/* Done, checked */

/**
 * Write enable (06h) sets the WEL bit in SR to 1. This bit
 * must be set prior to every Page Program, Sector Erase, 
 * Block Erase, Chip Erase and Write Status Register ins.
 * 
 * Dirive /CS low, shift instruction code to DI, Dirive /CS high 
*/
void w25q_write_enable(void);
/* Done, checked */

/**
 * Write disable (04h) resets the WEL bit in SR to a 0. WEL bit
 * resets automatically after pwr-up, write SR, Page Program and
 * Erase Instructions. Drive /CS low, shift instruction code 04h 
 * into a DI pin and then drive /CS high.
*/
void w25q_write_disable(void);
/* Done, checked */

/**
 * Read Status register instructions allow the 8-bit SR to be read.
 * Drive /CS low and shift instruction codes according to chosen SRx.
 * The 8 bits should be catched from DO pin <--MSB <--LSB.
 * 
 * Use this instruction for every time, even while a Program, Erase or
 * Write status register cycle is in progress.
*/
uint8_t w25q_status_reg_read(uint8_t SRx);
/* Done, checked */

/**
 * Write Status register allows the status register to be written. A
 * Write Enable instruction must be executed before sending write command.
 * Drive /CS low, send instruction code 01h, You can ONLY write 7,5,4,3,2
 * bits of SR_1 and 9, 8 bits of SR_2 
*/
void w25q_status_reg_write(uint8_t SRx, uint8_t reg_bits);
/* Not done */

/**
 * Read Data allows one or more data bytes to be sequentially read from the
 * memory. The instruction is initialized by driving /CS low, shifting 03h
 * code to the DI pin. frame is: 8-bit instruction, 24-bit address - 4 bytes.
 * After sending address you will receive DataOut <--MSB <--LSB on DO.
*/
void w25q_read_data(uint32_t address, uint8_t *databuf, uint8_t buff_size);
/* Done, not checked */

/**
 * Fast Read 0Bh as same as read data instruction, but it can operate on highest
 * frequencies Fr (see ds). Add 8 'dummy' clocks impulses after 24 bit address.
*/
void w25q_read_data_fast(uint8_t byte, uint32_t address);
/* Not done */

/**
 * Fast Read dual output (3Bh) - data is output on two pins.
*/
void w25q_read_data_fast_d(uint8_t byte_0, uint8_t data_1, uint32_t address);
/* Not done */

/**
 * Fast Read quad output 6Bh - data is output on 4 pins
*/
void w25q_read_data_fast_q(uint8_t byte_0, uint8_t byte_1,
                 uint8_t byte_2, uint8_t byte_3, uint32_t address);
/* Not Done */

/**
 * Fast Read dual I/O BBh. Allows for improved random accesss while maintaining
 * two IO pins. It can input the address bits 23-0 two bits per clock. DUAL SPI.
*/
void w25q_read_data_fast_dio(uint8_t byte_0, uint8_t byte_1, uint32_t address);
/* Not done */

/**
 * Fast Read Quad I/O (EBh) - same as dual I/O, but it used for QUAD SPI.
*/
void w25q_read_data_fast_qio(uint8_t byte_0, uint8_t byte_1,
                uint8_t byte_2, uint8_t byte_3, uint32_t address);
/* Not done */

/**
 * Word Read Quad I/O E7h insction: the lowest address bit A0 must equal 0
 * and only two Dummy clocks are required prior to the data output. Used
 * with QUAD SPI mode. The QE in SR-2 must be set to enable the WRQ I/0.
*/
void w25q_read_word_qio(uint8_t byte_0, uint32_t address);
/* Not done */

/**
 * Octal Word Read Quad I/O E3h. Lower A0-A3 must equal 0. 4 dummy clocks
 * are not required, which reduces the instuction overhead. QE in SR-2 must
 * be set to enable the WRQ I/O.
*/
void w25q_read_octal_qio(uint8_t* byte_0, uint32_t address);
/* Not done */

/**
 * The Page Program instruction (02h) allows from one byte to 256 bytes
 * (a page) of data to be programmed at previously erased (FFh) memory
 * locations. A Write Enable instruction must be executed before.
 * frame ex: Instruction <- 24-bit address <- DataByte ( 8 - 256 bits ).
 * 
 * If the data will be >255 bits, the IC will rewrite the page from start.
*/
void w25q_page_program(uint32_t address, uint8_t *databuf, uint8_t buff_size);
/* Done, not checked */

/**
 * Quad Input Page Program (32h) allows up to 256 bytes of data to be 
 * programmed at previously erased (FFh) memory locations using 4 pins.
 * This instruction may be useful with QUAD SPI applications.
*/
uint8_t w25q_page_program_q(uint8_t byte_0, uint32_t address);
/* Not done */

/**
 * The Sector Erase instruction sets all memory within a specified sector
 * 4K-bytes to the erased state of all 1s FFh. Write Enable instructions
 * must be executed before the device will accept the Sector Erase command.
 * Frame: Instruction <- 24-bit address (A23-A0).
*/
uint8_t w25q_erase_sector(uint8_t byte_0, uint32_t address);
/* Not done */

/**
 * Block Erase 32K-bytes. Check Block Protect Status Register to make sure
 * you can use this command. (SEC, TB, BP2, BP1, BP0).
*/
uint8_t w25q_erase_block_32k(uint8_t byte_0, uint32_t address);
/* Not done */

/**
 * Block Erase 64K-bytes sets all memory block to the erased state of all FF.
 * Use Write Enable before this instruction. This is the same instruction as
 * block erase 32K-bytes.
*/
uint8_t w25q_erase_block_64k(uint8_t byte_0, uint32_t address);
/* Not done */

/**
 * Chip Erase sets all memory within the device to the erased state of all 1s.
 * WE must be executed before. Check status register BP2, BP1, BP0 for allow.
*/
uint8_t w25q_erase_chip(uint8_t byte_0);
/* Not done */

/**
 * Erase Suspend 75h allows the system to interrupt a sector of block erase
 * operation and then read from or program data to, any other sectors/blocks.
 * For allowing this instruction  - bits SUS eq 0 and BUSY eq 1.
*/
uint8_t w25q_erase_suspend(uint8_t byte_0);
/* Not done */

/**
 * Erase Resume 7Ah must be written to resume the sector/block erase operation
 * after an Erase Suspend. The Resume instruction 7Ah will be accepted by the
 * device only if: SUS eq 1 & BUSY eq 0.
*/
uint8_t w25q_erase_resume(uint8_t byte_0);
/* Not done */

/**
 * Power-down B9h - reduce the power-down instruction.
*/
uint8_t w25q_power_dwn(uint8_t byte_0);
/* Not done */

/** Release Power-down / Deice-ID ABh is a multi-puprose instruction
 * It can be used to release the device from the power-down state or
 * obtain the devices electronic identification (ID) number.
*/
uint8_t w25q_power_dwn_release(uint8_t byte_0);
/* Not done */

/**
 * Read Manufacturer / Device-ID 90h is an alternative to the Release
 * from power-down / Device-ID instruction that provides both JEDEC
 * assigned manufacturer ID and the scecific device ID. It follows with
 * 90h command, A23-A0 address of 000000h - Manufacturer, Device ID. 2 bytes
 * 90h command, A23-A0 address of 000001h - Device ID, Manufacturer. 2 bytes
 * 
 * DUAL, QUADRO are also allowed with this IC.
*/
uint16_t w25q_read_manufacturer(void);
/* Done, not checked */

/**
 * Read unique ID number 4Bh accesses a factory-set read-only 64-bit number
 * that is unique to each W25Q16BV device. BIT-ID - 4 bits after dummy bits.
*/
uint8_t w25q_read_unique_id(uint8_t byte_0);
/* Not done */

/**
 * Read JEDEC ID 9Fh. identity of the device
*/
uint8_t w25q_read_jedec(uint8_t byte_0);
/* Not done */

/**
 * Continious Read Mode Reset FFh or FFFFh - for fast-read Dual/Quad I/O CRM
 * bits M7-M0.
*/
uint8_t w25q_cont_readmode_reset(uint8_t byte_0);
/* Not done */

#endif /* W25Q16BV_H*/

/* END OF FILE */
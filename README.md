The structure of this project was made with libopecm3 template. reference:
 https://github.com/libopencm3/libopencm3-template.git


 DEVICE: stm32f407ve6

 BOARD: https://stm32-base.org/boards/STM32F407VET6-STM32F4XX-M

 Documentation folder consists of <datasheet>.pdf for references

 # Instructions:
 1. git clone <linkaddress>
 2. make -C libopencm3  # use it once, build library object files
 3. cd ./core
 4. make firmware.elf   # OR make firmware.bin; just "make" for both

 # Directories
* core contatins source files applications and main.c file
* drivers contains shared files, modules, ex flash driver.


# How-to flash (st-link utilities)
 * After building project with make use this commands to flash
 * and communicate with device via minicom:
 * st-flash --reset --format binary write firmaware.bin 0x08000000
 * minicom --device /dev/ttyUSB0;
 * or:
 * dmesg | grep tty - check device connected ttyXXX
 * sudo municom -s -> configure device tty - save
 * !!! hardware and software flow control should be "NO"


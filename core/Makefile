PROJECT = firmware
BUILD_DIR = bin

SHARED_DIR = ../drivers
CFILES = main.c
CFILES += w25q16bv.c


DEVICE=STM32F407VG


VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
